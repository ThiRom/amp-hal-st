#include "hal_st/stm32fxxx/EthernetMacStm.hpp"
#include "generated/stm32fxxx/PeripheralTable.hpp"
#include "infra/event/EventDispatcher.hpp"
#include "infra/util/BitLogic.hpp"
#include "stm32h573xx.h"
#include <cstddef>
#include <cstdint>

#if defined(HAS_PERIPHERAL_ETHERNET)

/* Helper macros for RX descriptor handling */
#define INCR_RX_DESC_INDEX(inx, offset) do {\
                                             (inx) += (offset);\
                                             if ((inx) >= (uint32_t)ETH_RX_DESC_CNT){\
                                             (inx) = ((inx) - (uint32_t)ETH_RX_DESC_CNT);}\
                                           } while (0)

ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
ETH_HandleTypeDef   eth{};
ETH_HandleTypeDef   *heth{};
ETH_TxPacketConfigTypeDef TxConfig;
ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = { 0 };

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
        EnableClockEthernet(0);
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

        HAL_ETH_Start_IT(&eth);
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

        //     RequestReceiveBuffer();

        //     if (receiveDone)
        //     {
        //         if (!errorFrame)
        //             ethernetMac.GetObserver().ReceivedFrame(receivedFrameBuffers, frameSize);
        //         else
        //             ethernetMac.GetObserver().ReceivedErrorFrame(receivedFrameBuffers, frameSize);
        //         receivedFrameBuffers = 0;
        //     }
        // }
        bool receiveDone;
        do
        {
            receiveDone = RequestReceiveBuffer();
            if(receiveDone)
            {
                ++receivedFrameBuffers;
                uint16_t frameSize = heth->RxDescList.RxDataLength; //RT: Check!!
                ethernetMac.GetObserver().ReceivedFrame(receivedFrameBuffers, frameSize);
                receivedFrameBuffers = 0;
            }
        }
        while(receiveDone);
    }

    void EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffers()
    {
        // while (receivedFramesAllocated != descriptors.size())
        //     if (!RequestReceiveBuffer())
        //         break;

        RequestReceiveBuffer();
    }

    /**
    * @brief  This function gives back Rx Desc of the last received Packet
    *         to the DMA, so ETH DMA will be able to use these descriptors
    *         to receive next Packets.
    * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
    *         the configuration information for ETHERNET module
    * @retval HAL status
    */
    void EthernetMacStm::ReceiveDescriptors::ETH_UpdateDescriptor(ETH_HandleTypeDef *heth)
    {
        uint32_t descidx;
        uint32_t tailidx;
        uint32_t desccount;
        ETH_DMADescTypeDef *dmarxdesc;
        infra::ByteRange buff;
        uint8_t allocStatus = 1U;

        descidx = heth->RxDescList.RxBuildDescIdx;
        dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
        desccount = heth->RxDescList.RxBuildDescCnt;

        while ((desccount > 0U) && (allocStatus != 0U))
        {
            /* Check if a buffer's attached the descriptor */
            if (READ_REG(dmarxdesc->BackupAddr0) == 0U)
            {
                /* Get a new buffer. */
                buff = ethernetMac.GetObserver().RequestReceiveBuffer();

                if (buff.empty())
                {
                    allocStatus = 0U;
                }
                else
                {
                    WRITE_REG(dmarxdesc->BackupAddr0, reinterpret_cast<uint32_t>(buff.begin()));
                    WRITE_REG(dmarxdesc->DESC0, reinterpret_cast<uint32_t>(buff.begin()));
                }
            }

            if (allocStatus != 0U)
            {

                if (heth->RxDescList.ItMode != 0U)
                {
                    WRITE_REG(dmarxdesc->DESC3, ETH_DMARXNDESCRF_OWN | ETH_DMARXNDESCRF_BUF1V | ETH_DMARXNDESCRF_IOC);
                }
                else
                {
                    WRITE_REG(dmarxdesc->DESC3, ETH_DMARXNDESCRF_OWN | ETH_DMARXNDESCRF_BUF1V);
                }

                /* Increment current rx descriptor index */
                INCR_RX_DESC_INDEX(descidx, 1U);
                /* Get current descriptor address */
                dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
                desccount--;
            }
        }

        if (heth->RxDescList.RxBuildDescCnt != desccount)
        {
            /* Set the tail pointer index */
            tailidx = (descidx + 1U) % ETH_RX_DESC_CNT;

            /* DMB instruction to avoid race condition */
            __DMB();

            /* Set the Tail pointer address */
            WRITE_REG(heth->Instance->DMACRDTPR, ((uint32_t)(heth->Init.RxDesc + (tailidx))));

            heth->RxDescList.RxBuildDescIdx = descidx;
            heth->RxDescList.RxBuildDescCnt = desccount;
        }
    }

    bool EthernetMacStm::ReceiveDescriptors::RequestReceiveBuffer()
    {
        // RT: copied from readdata of HAL_ETH

        uint32_t descidx;
        ETH_DMADescTypeDef *dmarxdesc;
        uint32_t desccnt = 0U;
        uint32_t desccntmax;
        uint32_t bufflength;
        uint8_t rxdataready = 0U;

        if (heth->gState != HAL_ETH_STATE_STARTED)
        {
            return false;
        }

        descidx = heth->RxDescList.RxDescIdx;
        dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
        desccntmax = ETH_RX_DESC_CNT - heth->RxDescList.RxBuildDescCnt;

        /* Check if descriptor is not owned by DMA */
        while ((READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_OWN) == (uint32_t)RESET) && (desccnt < desccntmax)
                && (rxdataready == 0U))
        {
            if (READ_BIT(dmarxdesc->DESC3,  ETH_DMARXNDESCWBF_CTXT)  != (uint32_t)RESET)
            {
                /* Get timestamp high */
                heth->RxDescList.TimeStamp.TimeStampHigh = dmarxdesc->DESC1;
                /* Get timestamp low */
                heth->RxDescList.TimeStamp.TimeStampLow  = dmarxdesc->DESC0;
            }
            if ((READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_FD) != (uint32_t)RESET) || (heth->RxDescList.pRxStart != NULL))
            {
                /* Check if first descriptor */
                if (READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_FD) != (uint32_t)RESET)
                {
                    heth->RxDescList.RxDescCnt = 0;
                    heth->RxDescList.RxDataLength = 0;
                }

                /* Get the Frame Length of the received packet: substruct 4 bytes of the CRC */
                bufflength = READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_PL) - heth->RxDescList.RxDataLength;

                /* Check if last descriptor */
                if (READ_BIT(dmarxdesc->DESC3, ETH_DMARXNDESCWBF_LD) != (uint32_t)RESET)
                {
                    /* Save Last descriptor index */
                    heth->RxDescList.pRxLastRxDesc = dmarxdesc->DESC3;

                    /* Packet ready */
                    rxdataready = 1;
                }

                /* Link data */

                heth->RxDescList.RxDescCnt++;
                heth->RxDescList.RxDataLength += bufflength;

                /* Clear buffer pointer */
                dmarxdesc->BackupAddr0 = 0;
            }

            /* Increment current rx descriptor index */
            INCR_RX_DESC_INDEX(descidx, 1U);
            /* Get current descriptor address */
            dmarxdesc = (ETH_DMADescTypeDef *)heth->RxDescList.RxDesc[descidx];
            desccnt++;
        }

        heth->RxDescList.RxBuildDescCnt += desccnt;
        if ((heth->RxDescList.RxBuildDescCnt) != 0U)
        {
            /* Update Descriptors */
            ETH_UpdateDescriptor(heth);
        }

        heth->RxDescList.RxDescIdx = descidx;

        if (rxdataready == 1U)
        {
            /* Return received packet */
            //*pAppBuff = heth->RxDescList.pRxStart;
            /* Reset first element */
            heth->RxDescList.pRxStart = NULL;

            return true;
        }

        /* Packet not ready */
        return false;
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
        memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));

        Txbuffer[0].buffer = (uint8_t*)data.begin();
		Txbuffer[0].len = data.size() + 1;
        Txbuffer[0].next = NULL;

        TxConfig.Length = data.size() + 1;
        TxConfig.TxBuffer = Txbuffer;
        TxConfig.pData = (uint8_t*)data.begin(); //?

        HAL_ETH_Transmit_IT(heth, &TxConfig);

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
