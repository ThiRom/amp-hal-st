#include "hal_st/stm32fxxx/EthernetMacStm.hpp"
#include "infra/event/EventDispatcher.hpp"
#include "infra/util/BitLogic.hpp"
#include "stm32h573xx.h"
#include <cstddef>
#include <cstdint>

#if defined(HAS_PERIPHERAL_ETHERNET)

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
ETH_HandleTypeDef   eth{};
ETH_HandleTypeDef   *heth{};
ETH_TxPacketConfigTypeDef TxConfig;

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
        peripheralEthernet[0]->MACA0LR = reinterpret_cast<const uint32_t*>(macAddress.data())[0];
        peripheralEthernet[0]->MACA0HR = reinterpret_cast<const uint32_t*>(macAddress.data())[1] & 0xffff;

        static uint8_t MACAddr[6];
        heth = &eth;

        eth.Instance = peripheralEthernet[0];
        MACAddr[0] = 0x00;
        MACAddr[1] = 0x80;
        MACAddr[2] = 0xE1;
        MACAddr[3] = 0x00;
        MACAddr[4] = 0x00;
        MACAddr[5] = 0x01;
        eth.Init.MACAddr = &MACAddr[0];
        eth.Init.MediaInterface = HAL_ETH_RMII_MODE;
        eth.Init.TxDesc = DMATxDscrTab;
        eth.Init.RxDesc = DMARxDscrTab;
        eth.Init.RxBuffLen = 1524;

        HAL_ETH_Init(&eth);

        memset(&TxConfig, 0, sizeof(ETH_TxPacketConfigTypeDef));
	    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
    }

    EthernetMacStm::~EthernetMacStm()
    {
        ResetDma();
        peripheralEthernet[0]->MACCR = 0;
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
        // // Normal interrupt summary
        // if ((peripheralEthernet[0]->DMASR & ETH_DMASR_NIS) != 0)
        // {
        //     peripheralEthernet[0]->DMASR = ETH_DMASR_NIS;
        //     // Transmit status
        //     if ((peripheralEthernet[0]->DMASR & ETH_DMASR_TS) != 0)
        //     {
        //         peripheralEthernet[0]->DMASR = ETH_DMASR_TS;
        //         sendDescriptors.SentFrame();
        //     }

        //     // Receive status
        //     if ((peripheralEthernet[0]->DMASR & ETH_DMASR_RS) != 0)
        //     {
        //         peripheralEthernet[0]->DMASR = ETH_DMASR_RS;
        //         receiveDescriptors.ReceivedFrame();
        //     }
        // }

        // // Abnormal interrupt summary
        // if ((peripheralEthernet[0]->DMASR & ETH_DMASR_AIS) != 0)
        // {
        //     // Receiver process stopped: Indicates an error in our logic
        //     if ((peripheralEthernet[0]->DMASR & ETH_DMASR_RPSS) != 0)
        //         std::abort();

        //     // Fatal bus error by ethernet DMA: Indicates an error in setting up descriptors
        //     if ((peripheralEthernet[0]->DMASR & ETH_DMASR_FBES) != 0)
        //         std::abort();

        //     peripheralEthernet[0]->DMASR = ETH_DMASR_AIS;
        // }

        uint32_t mac_flag = READ_REG(heth->Instance->MACISR);
        uint32_t dma_flag = READ_REG(heth->Instance->DMACSR);
        uint32_t dma_itsource = READ_REG(heth->Instance->DMACIER);
        uint32_t exti_flag = READ_REG(EXTI->RPR2);

        /* Packet received */
        if (((dma_flag & ETH_DMACSR_RI) != 0U) && ((dma_itsource & ETH_DMACIER_RIE) != 0U))
        {
            /* Clear the Eth DMA Rx IT pending bits */
            __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMACSR_RI | ETH_DMACSR_NIS);

            sendDescriptors.SentFrame();
        }

        /* Packet transmitted */
        if (((dma_flag & ETH_DMACSR_TI) != 0U) && ((dma_itsource & ETH_DMACIER_TIE) != 0U))
        {
            /* Clear the Eth DMA Tx IT pending bits */
            __HAL_ETH_DMA_CLEAR_IT(heth, ETH_DMACSR_TI | ETH_DMACSR_NIS);

            receiveDescriptors.ReceivedFrame();
        }
    }

    EthernetMacStm::ReceiveDescriptors::ReceiveDescriptors(EthernetMacStm& ethernetMac)
        : ethernetMac(ethernetMac)
    {
        // for (auto& descriptor : descriptors)
        // {
        //     descriptor.DESC0 = ETH_DMARXDESC_RCH;
        //     descriptor.DESC1 = 0;
        //     descriptor.DESC3 = reinterpret_cast<uint32_t>(&descriptor + 1);
        //     descriptor.DESC4 = 0;
        // }
        // descriptors.back().DESC1 |= ETH_DMARXDESC_RER;
        // descriptors.back().DESC3 = reinterpret_cast<uint32_t>(&descriptors.front());

        // peripheralEthernet[0]->DMARDLAR = reinterpret_cast<uint32_t>(descriptors.data());

        infra::EventDispatcher::Instance().Schedule([this]()
            {
                // This is scheduled so that the observer is instantiated
                RequestReceiveBuffers();
            });
    }

    void EthernetMacStm::ReceiveDescriptors::ReceivedFrame()
    {
        // while (receivedFramesAllocated != 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_OWN) == 0)
        // {
        //     bool receiveDone = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_OWN) == 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_LS) != 0;
        //     uint16_t frameSize = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_FL) >> 16;
        //     bool errorFrame = (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_ES) != 0 && (descriptors[receiveDescriptorReceiveIndex].DESC0 & ETH_DMARXDESC_LS) != 0;
        //     descriptors[receiveDescriptorReceiveIndex].DESC2 = 0;
        //     ++receivedFrameBuffers;
        //     --receivedFramesAllocated;
        //     ++receiveDescriptorReceiveIndex;
        //     if (receiveDescriptorReceiveIndex == descriptors.size())
        //         receiveDescriptorReceiveIndex = 0;
        //      RequestReceiveBuffer();
        //     if (receiveDone)
        //     {
        //         if (!errorFrame)
        //             ethernetMac.GetObserver().ReceivedFrame(receivedFrameBuffers, frameSize);
        //         else
        //             ethernetMac.GetObserver().ReceivedErrorFrame(receivedFrameBuffers, frameSize);
        //         receivedFrameBuffers = 0;
        //     }
        // }

        void* p;
        do
        {
            p = RequestReceiveBuffer();
            if(p != nullptr)
            {
                ++receivedFrameBuffers;
                uint16_t frameSize = heth->RxDescList.RxDataLength; //RT: Check!!
                ethernetMac.GetObserver().ReceivedFrame(receivedFrameBuffers, frameSize);
            }
        }
        while(p != nullptr);
    }

    void EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffers()
    {
        // while (receivedFramesAllocated != descriptors.size())
        //     if (!RequestReceiveBuffer())
        //         break;

        RequestReceiveBuffer();
    }

    void* EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffer()
    {
        // assert((descriptors[receiveDescriptorAllocatedIndex].DESC0 & ETH_DMARXDESC_OWN) == 0);

        void* buffer = ethernetMac.GetObserver().RequestReceiveBuffer();
        if (buffer == nullptr)
             return buffer;

        HAL_ETH_ReadData(heth, (void**)&buffer);

        // descriptors[receiveDescriptorAllocatedIndex].DESC0 &= ~(ETH_DMARXDESC_MAMPCE | ETH_DMARXDESC_CE | ETH_DMARXDESC_DBE | ETH_DMARXDESC_RE | ETH_DMARXDESC_RWT | ETH_DMARXDESC_LC | ETH_DMARXDESC_IPV4HCE | ETH_DMARXDESC_LS | ETH_DMARXDESC_VLAN | ETH_DMARXDESC_OE | ETH_DMARXDESC_LE | ETH_DMARXDESC_SAF | ETH_DMARXDESC_DE | ETH_DMARXDESC_ES | ETH_DMARXDESC_FL | ETH_DMARXDESC_AFM);
        // descriptors[receiveDescriptorAllocatedIndex].DESC1 = buffer.size() | ETH_DMARXDESC_RCH;
        // descriptors[receiveDescriptorAllocatedIndex].DESC2 = reinterpret_cast<uint32_t>(buffer.begin());
        // descriptors[receiveDescriptorAllocatedIndex].DESC0 |= ETH_DMARXDESC_OWN;

        // __DSB();
        // peripheralEthernet[0]->DMASR = ETH_DMASR_RBUS;
        // peripheralEthernet[0]->DMARPDR = 1;

        // ++receivedFramesAllocated;
        // ++receiveDescriptorAllocatedIndex;
        // if (receiveDescriptorAllocatedIndex == descriptors.size())
        //     receiveDescriptorAllocatedIndex = 0;

        return buffer;
    }

    EthernetMacStm::SendDescriptors::SendDescriptors(EthernetMacStm& ethernetMac)
        : ethernetMac(ethernetMac)
    {
        // for (auto& descriptor : descriptors)
        // {
        //     descriptor.DESC0 = ETH_DMATXDESC_TCH | ETH_DMATXDESC_CIC_TCPUDPICMP_FULL | ETH_DMATXDESC_IC;
        //     descriptor.DESC3 = reinterpret_cast<uint32_t>(&descriptor + 1);
        // }
        // descriptors.back().DESC0 |= ETH_DMATXDESC_TER;
        // descriptors.back().DESC3 = reinterpret_cast<uint32_t>(&descriptors.front());

        // peripheralEthernet[0]->DMATDLAR = reinterpret_cast<uint32_t>(descriptors.data());
    }

    void EthernetMacStm::SendDescriptors::SendBuffer(infra::ConstByteRange data, bool last)
    {
        ETH_BufferTypeDef buf;

        buf.buffer = (uint8_t*)&data;
        buf.len = data.size();
        buf.next = NULL;

        TxConfig.Length = data.size();
        TxConfig.TxBuffer = &buf;

        HAL_ETH_Transmit(heth, &TxConfig, 20); //20 msec timeout

        // assert((descriptors[sendDescriptorIndex].DESC0 & ETH_DMATXDESC_OWN) == 0);
        // descriptors[sendDescriptorIndex].DESC1 = data.size();
        // descriptors[sendDescriptorIndex].DESC2 = reinterpret_cast<uint32_t>(data.begin());

        // if (sendFirst)
        //     descriptors[sendDescriptorIndex].DESC0 |= ETH_DMATXDESC_FS;
        // else
        //     descriptors[sendDescriptorIndex].DESC0 &= ~ETH_DMATXDESC_FS;

        // if (last)
        //     descriptors[sendDescriptorIndex].DESC0 |= ETH_DMATXDESC_LS;
        // else
        //     descriptors[sendDescriptorIndex].DESC0 &= ~ETH_DMATXDESC_LS;

        // descriptors[sendDescriptorIndex].DESC0 &= ~(ETH_DMATXDESC_DB | ETH_DMATXDESC_UF | ETH_DMATXDESC_ED | ETH_DMATXDESC_CC | ETH_DMATXDESC_EC | ETH_DMATXDESC_LCO | ETH_DMATXDESC_NC | ETH_DMATXDESC_LCA | ETH_DMATXDESC_PCE | ETH_DMATXDESC_FF | ETH_DMATXDESC_JT | ETH_DMATXDESC_ES | ETH_DMATXDESC_IHE);

        // if (sendFirst)
        //     sendDescriptorIndexFirst = sendDescriptorIndex;
        // else
        //     descriptors[sendDescriptorIndex].DESC0 |= ETH_DMATXDESC_OWN;
        // if (last)
        //     descriptors[sendDescriptorIndexFirst].DESC0 |= ETH_DMATXDESC_OWN;
        // __DSB();
        // sendFirst = last;

        // peripheralEthernet[0]->DMATPDR = 1;

        // ++sendDescriptorIndex;
        // if (sendDescriptorIndex == descriptors.size())
        //     sendDescriptorIndex = 0;
    }

    void EthernetMacStm::SendDescriptors::SentFrame()
    {
        // uint32_t previousDescriptor = sendDescriptorIndex != 0 ? sendDescriptorIndex - 1 : descriptors.size() - 1;

        // bool sentDone = (descriptors[previousDescriptor].DESC0 & ETH_DMATXDESC_LS) != 0 && (descriptors[previousDescriptor].DESC0 & ETH_DMATXDESC_OWN) == 0;
        // assert(sentDone);
        // if (sentDone)
        // {
        //     descriptors[previousDescriptor].DESC0 &= ~ETH_DMATXDESC_LS;
             ethernetMac.GetObserver().SentFrame();
        // }
    }
}

#endif
