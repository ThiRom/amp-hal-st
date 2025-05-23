#include "hal_st/stm32fxxx/EthernetMacStm.hpp"
#include "infra/event/EventDispatcher.hpp"
#include "infra/util/BitLogic.hpp"

#include "stm32h573xx.h"
#include "stm32h5xx_hal_eth.h"

#if defined(HAS_PERIPHERAL_ETHERNET)

#define ETH_SEGMENT_SIZE_DEFAULT        0x218U
#define ETH_DMACTCR_TPBL_32PBL          ((uint32_t)0x00200000)   /* Transmit Programmable Burst Length 32 */
#define ETH_DMACRCR_RPBL_32PBL          ((uint32_t)0x00200000)   /* Receive Programmable Burst Length 32 */
#define ETH_MACCR_MASK                  0xFFFB7F7CU

namespace hal
{
    EthernetMacStm::EthernetMacStm(EthernetSmi& ethernetSmi, LinkSpeed linkSpeed, std::array<uint8_t, 6> macAddress)
        : ethernetSmi(ethernetSmi)
        , macAddress(macAddress)
        , interrupt(peripheralEthernetIrq[0], [this]()
              {
                  Interrupt();
              })
        , receiveDescriptors(*this)
        , sendDescriptors(*this)
    {
        uint32_t tickstart;
        uint32_t hclk;
        uint32_t tmpreg;

        EnableClockEthernet(0);

        // Select RMII Mode
        MODIFY_REG(SBS->PMCR, SBS_PMCR_ETH_SEL_PHY, (uint32_t)(SBS_ETH_RMII));
        // Dummy read to sync with ETH
        (void)SBS->PMCR;

        // Get the ETHERNET MACMDIOAR value
        tmpreg = peripheralEthernet[0]->MACMDIOAR;
        // Clear CSR Clock Range bits
        tmpreg &= ~ETH_MACMDIOAR_CR;
        // Get hclk frequency value
        hclk = HAL_RCC_GetHCLKFreq();
        // Set CR bits depending on hclk value
        if (hclk < 35000000U)
        {
            // CSR Clock Range between 0-35 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV16;
        }
        else if (hclk < 60000000U)
        {
            // CSR Clock Range between 35-60 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV26;
        }
        else if (hclk < 100000000U)
        {
            // CSR Clock Range between 60-100 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV42;
        }
        else if (hclk < 150000000U)
        {
            // CSR Clock Range between 100-150 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV62;
        }
        else if (hclk < 250000000U)
        {
            // CSR Clock Range between 150-250 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV102;
        }
        else // (hclk >= 250000000U)
        {
            // CSR Clock >= 250 MHz
            tmpreg |= ETH_MACMDIOAR_CR_DIV124;
        }
        // Configure the CSR Clock Range
        peripheralEthernet[0]->MACMDIOAR |= tmpreg;

        // MAC LPI 1US Tic Counter Configuration
        peripheralEthernet[0]->MAC1USTCR |= (((uint32_t)HAL_RCC_GetHCLKFreq() / ETH_MAC_US_TICK) - 1U);

        // Set MAC address
        peripheralEthernet[0]->MACA0LR = reinterpret_cast<const uint32_t*>(macAddress.data())[0];
        peripheralEthernet[0]->MACA0HR = reinterpret_cast<const uint32_t*>(macAddress.data())[1] & 0xffff;

        //Set default MAC settings like in HAL_ETH see ETH_MACDMAConfig()
        peripheralEthernet[0]->MACCR |= ((linkSpeed == LinkSpeed::fullDuplex100MHz || linkSpeed == LinkSpeed::halfDuplex100MHz) ? ETH_MACCR_FES : 0) |
                                        ((linkSpeed == LinkSpeed::fullDuplex100MHz || linkSpeed == LinkSpeed::fullDuplex10MHz) ? ETH_MACCR_DM : 0) |
                                        ETH_MACCR_SARC_REPADDR0 | ETH_MACCR_CST | ETH_MACCR_IPC | ETH_MACCR_JD | ETH_MACCR_TE | ETH_MACCR_RE;

        //Set Transmit Store and Forward
        peripheralEthernet[0]->MTLTQOMR |= ETH_MTLTQOMR_TSF;

        //Set default DMA settings like in HAL_ETH see ETH_MACDMAConfig()
        peripheralEthernet[0]->DMASBMR |= ETH_DMASBMR_AAL;
        peripheralEthernet[0]->DMACCR |= ETH_SEGMENT_SIZE_DEFAULT | ETH_DMACCR_DSL_64BIT;
        peripheralEthernet[0]->DMACTCR |= ETH_DMACTCR_TPBL_32PBL;
        peripheralEthernet[0]->DMACRCR |= ETH_DMACRCR_RPBL_32PBL;

        //Enable interrupts
        peripheralEthernet[0]->DMACIER |= ETH_DMACIER_TIE | ETH_DMACIER_RIE | ETH_DMACIER_RSE | ETH_DMACIER_FBEE | ETH_DMACIER_AIE | ETH_DMACIER_NIE;

        // Disable Rx MMC Interrupts (mask register -> 1 = OFF)
        peripheralEthernet[0]->MMCRIMR |= ETH_MMCRIMR_RXLPITRCIM | ETH_MMCRIMR_RXLPIUSCIM | ETH_MMCRIMR_RXUCGPIM | ETH_MMCRIMR_RXALGNERPIM | ETH_MMCRIMR_RXCRCERPIM;

        // Disable Tx MMC Interrupts (mask register -> 1 = OFF)
        peripheralEthernet[0]->MMCTIMR |= ETH_MMCTIMR_TXLPITRCIM | ETH_MMCTIMR_TXLPIUSCIM | ETH_MMCTIMR_TXGPKTIM | ETH_MMCTIMR_TXMCOLGPIM | ETH_MMCTIMR_TXSCOLGPIM;
    }

    EthernetMacStm::~EthernetMacStm()
    {
        ResetDma();
        MODIFY_REG(peripheralEthernet[0]->MACCR, ETH_MACCR_MASK, 0);
    }

    void EthernetMacStm::SendBuffer(infra::ConstByteRange data, bool last)
    {
        sendDescriptors.SendBuffer(data, last);
    }

    void EthernetMacStm::RetryAllocation()
    {
        receiveDescriptors.RequestReceiveBuffers();
    }

    void EthernetMacStm::AddMacAddressFilter(MacAddress address)
    {
        uint32_t lr = reinterpret_cast<const uint32_t*>(address.data())[0];
        uint32_t hr = (reinterpret_cast<const uint32_t*>(address.data())[1] & 0xffff) | (1 << 31);

        if ((peripheralEthernet[0]->MACA1HR & infra::Bit<uint32_t>(31)) == 0)
        {
            peripheralEthernet[0]->MACA1LR = lr;
            peripheralEthernet[0]->MACA1HR = hr;
        }
        else if ((peripheralEthernet[0]->MACA2HR & infra::Bit<uint32_t>(31)) == 0)
        {
            peripheralEthernet[0]->MACA2LR = lr;
            peripheralEthernet[0]->MACA2HR = hr;
        }
        else if ((peripheralEthernet[0]->MACA3HR & infra::Bit<uint32_t>(31)) == 0)
        {
            peripheralEthernet[0]->MACA3LR = lr;
            peripheralEthernet[0]->MACA3HR = hr;
        }
        else
        {
            // No free mac address found. Hint: implement address hashing
            abort();
        }
    }

    void EthernetMacStm::RemoveMacAddressFilter(MacAddress address)
    {
        uint32_t lr = reinterpret_cast<const uint32_t*>(address.data())[0];
        uint32_t hr = (reinterpret_cast<const uint32_t*>(address.data())[1] & 0xffff) | (1 << 31);

        if (peripheralEthernet[0]->MACA1HR == hr && peripheralEthernet[0]->MACA1LR == lr)
        {
            peripheralEthernet[0]->MACA1LR = 0;
            peripheralEthernet[0]->MACA1HR = 0;
        }
        else if (peripheralEthernet[0]->MACA2HR == hr && peripheralEthernet[0]->MACA2LR == lr)
        {
            peripheralEthernet[0]->MACA2LR = 0;
            peripheralEthernet[0]->MACA2HR = 0;
        }
        else if (peripheralEthernet[0]->MACA3HR == hr && peripheralEthernet[0]->MACA3LR == lr)
        {
            peripheralEthernet[0]->MACA3LR = 0;
            peripheralEthernet[0]->MACA3HR = 0;
        }
        else
        {
            // Address not found
            abort();
        }
    }

    void EthernetMacStm::ResetDma()
    {
        peripheralEthernet[0]->DMAMR |= ETH_DMAMR_SWR;
        while ((peripheralEthernet[0]->DMAMR & ETH_DMAMR_SWR) != 0)
        {}
    }

    void EthernetMacStm::Interrupt()
    {
        // Normal interrupt summary
        if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_NIS) != 0)
        {
            peripheralEthernet[0]->DMACSR = ETH_DMACSR_NIS;
            // Transmit status
            if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_TI) != 0)
            {
                peripheralEthernet[0]->DMACSR = ETH_DMACSR_TI;
                sendDescriptors.SentFrame();
            }

            // Receive status
            if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_RI) != 0)
            {
                peripheralEthernet[0]->DMACSR = ETH_DMACSR_RI;
                receiveDescriptors.ReceivedFrame();
            }
        }

        // Abnormal interrupt summary
        if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_AIS) != 0)
        {
            // Receiver process stopped: Indicates an error in our logic
            if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_RPS) != 0)
                std::abort();

            // Fatal bus error by ethernet DMA: Indicates an error in setting up descriptors
            if ((peripheralEthernet[0]->DMACSR & ETH_DMACSR_FBE) != 0)
                std::abort();

            peripheralEthernet[0]->DMACSR = ETH_DMACSR_AIS;
        }
    }

    EthernetMacStm::ReceiveDescriptors::ReceiveDescriptors(EthernetMacStm& ethernetMac)
        : ethernetMac(ethernetMac)
    {
        for (auto& descriptor : descriptors)
        {
            descriptor.DESC0 = ETH_DMARXDESC_RCH;
            descriptor.DESC1 = 0;
            descriptor.DESC3 = reinterpret_cast<uint32_t>(&descriptor + 1);
            descriptor.DESC4 = 0;
        }
        descriptors.back().DESC1 |= ETH_DMARXDESC_RER;
        descriptors.back().DESC3 = reinterpret_cast<uint32_t>(&descriptors.front());

        peripheralEthernet[0]->DMARDLAR = reinterpret_cast<uint32_t>(descriptors.data());

        infra::EventDispatcher::Instance().Schedule([this]()
            {
                // This is scheduled so that the observer is instantiated
                RequestReceiveBuffers();
            });
    }

    void EthernetMacStm::ReceiveDescriptors::ReceivedFrame()
    {
        while (receivedFramesAllocated != 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_OWN) == 0)
        {
            bool receiveDone = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_OWN) == 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_LS) != 0;
            uint16_t frameSize = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_FL) >> 16;
            bool errorFrame = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_ES) != 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_LS) != 0;
            descriptors[receiveDescriptorReceiveIndex].DESC2 = 0;
            ++receivedFrameBuffers;
            --receivedFramesAllocated;
            ++receiveDescriptorReceiveIndex;
            if (receiveDescriptorReceiveIndex == descriptors.size())
                receiveDescriptorReceiveIndex = 0;

            RequestReceiveBuffer();

            if (receiveDone)
            {
                if (!errorFrame)
                    ethernetMac.GetObserver().ReceivedFrame(receivedFrameBuffers, frameSize);
                else
                    ethernetMac.GetObserver().ReceivedErrorFrame(receivedFrameBuffers, frameSize);
                receivedFrameBuffers = 0;
            }
        }
    }

    void EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffers()
    {
        while (receivedFramesAllocated != descriptors.size())
            if (!RequestReceiveBuffer())
                break;
    }

    bool EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffer()
    {
        assert((descriptors[receiveDescriptorAllocatedIndex].DESC0 & ETH_DMARXDESC_OWN) == 0);

        infra::ByteRange buffer = ethernetMac.GetObserver().RequestReceiveBuffer();
        if (buffer.empty())
            return false;

        descriptors[receiveDescriptorAllocatedIndex].DESC0 &= ~(ETH_DMARXDESC_MAMPCE | ETH_DMARXDESC_CE | ETH_DMARXDESC_DBE | ETH_DMARXDESC_RE | ETH_DMARXDESC_RWT | ETH_DMARXDESC_LC | ETH_DMARXDESC_IPV4HCE | ETH_DMARXDESC_LS | ETH_DMARXDESC_VLAN | ETH_DMARXDESC_OE | ETH_DMARXDESC_LE | ETH_DMARXDESC_SAF | ETH_DMARXDESC_DE | ETH_DMARXDESC_ES | ETH_DMARXDESC_FL | ETH_DMARXDESC_AFM);
        descriptors[receiveDescriptorAllocatedIndex].DESC1 = buffer.size() | ETH_DMARXDESC_RCH;
        descriptors[receiveDescriptorAllocatedIndex].DESC2 = reinterpret_cast<uint32_t>(buffer.begin());
        descriptors[receiveDescriptorAllocatedIndex].DESC0 |= ETH_DMARXDESC_OWN;

        __DSB();
        peripheralEthernet[0]->DMASR = ETH_DMASR_RBUS;
        peripheralEthernet[0]->DMARPDR = 1;

        ++receivedFramesAllocated;
        ++receiveDescriptorAllocatedIndex;
        if (receiveDescriptorAllocatedIndex == descriptors.size())
            receiveDescriptorAllocatedIndex = 0;

        return true;
    }

    EthernetMacStm::SendDescriptors::SendDescriptors(EthernetMacStm& ethernetMac)
        : ethernetMac(ethernetMac)
    {
        for (auto& descriptor : descriptors)
        {
            descriptor.DESC3 = ETH_DMATXNDESCRF_CIC_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
        }

        //Set Channel Tx descriptor list address register
        peripheralEthernet[0]->DMACTDLAR = reinterpret_cast<uint32_t>(&descriptors[0]);
        //Set tail pointer to first element
        peripheralEthernet[0]->DMACTDTPR = reinterpret_cast<uint32_t>(&descriptors[0]);
        //Set Channel Tx descriptor ring length register
        peripheralEthernet[0]->DMACTRLR = 12;
    }

    void EthernetMacStm::SendDescriptors::SendBuffer(infra::ConstByteRange data, bool last)
    {
        assert((descriptors[sendDescriptorIndex].DESC3 & ETH_DMATXNDESCRF_OWN) == 0);
        descriptors[sendDescriptorIndex].DESC2 = data.size() | ETH_DMATXNDESCRF_IOC; // Set IOC bit here
        descriptors[sendDescriptorIndex].DESC0 = reinterpret_cast<uint32_t>(data.begin());

        if (sendFirst)
            descriptors[sendDescriptorIndex].DESC3 |= ETH_DMATXNDESCRF_FD;
        else
            descriptors[sendDescriptorIndex].DESC3 &= ~ETH_DMATXNDESCRF_FD;

        if (last)
            descriptors[sendDescriptorIndex].DESC3 |= ETH_DMATXNDESCRF_LD;
        else
            descriptors[sendDescriptorIndex].DESC3 &= ~ETH_DMATXNDESCRF_LD;

        //Clear status bits
        descriptors[sendDescriptorIndex].DESC3 &= ~(ETH_DMATXNDESCWBF_DB | ETH_DMATXNDESCWBF_UF | ETH_DMATXNDESCWBF_ED | ETH_DMATXNDESCWBF_CC | ETH_DMATXNDESCWBF_EC | ETH_DMATXNDESCWBF_LCO | ETH_DMATXNDESCWBF_NC | ETH_DMATXNDESCWBF_LCA | ETH_DMATXNDESCWBF_PCE | ETH_DMATXNDESCWBF_FF | ETH_DMATXNDESCWBF_JT | ETH_DMATXNDESCWBF_ES | ETH_DMATXNDESCWBF_IHE);

        if (sendFirst)
            sendDescriptorIndexFirst = sendDescriptorIndex;
        else
            descriptors[sendDescriptorIndex].DESC3 |= ETH_DMATXNDESCRF_OWN;
        if (last)
            descriptors[sendDescriptorIndexFirst].DESC3 |= ETH_DMATXNDESCRF_OWN;
        __DSB();
        sendFirst = last;

        ++sendDescriptorIndex;
        if (sendDescriptorIndex == descriptors.size())
            sendDescriptorIndex = 0;

        //Updating the tail pointer will issue a poll request
        peripheralEthernet[0]->DMACTDTPR = reinterpret_cast<uint32_t>(&descriptors[sendDescriptorIndex]);
    }

    void EthernetMacStm::SendDescriptors::SentFrame()
    {
        uint32_t previousDescriptor = sendDescriptorIndex != 0 ? sendDescriptorIndex - 1 : descriptors.size() - 1;

        bool sentDone = (descriptors[previousDescriptor].DESC3 & ETH_DMATXNDESCRF_FD) != 0 && (descriptors[previousDescriptor].DESC3 & ETH_DMATXCDESC_OWN) == 0;
        assert(sentDone);
        if (sentDone)
        {
            descriptors[previousDescriptor].DESC3 &= ~ETH_DMATXNDESCRF_LD;
            ethernetMac.GetObserver().SentFrame();
        }
    }
}

#endif
