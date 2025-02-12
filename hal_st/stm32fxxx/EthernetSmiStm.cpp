#include "hal_st/stm32fxxx/EthernetSmiStm.hpp"
#include "stm32h5xx_hal_eth.h"
#include DEVICE_HEADER
#include "generated/stm32fxxx/PeripheralTable.hpp"
#include "infra/util/BitLogic.hpp"

#if defined(HAS_PERIPHERAL_ETHERNET)

namespace hal
{
    EthernetSmiStm::EthernetSmiStm(hal::GpioPinStm& ethernetMdio, hal::GpioPinStm& ethernetMdc, hal::GpioPinStm& ethernetRmiiRefClk,
        hal::GpioPinStm& ethernetRmiiCrsDv, hal::GpioPinStm& ethernetRmiiRxD0, hal::GpioPinStm& ethernetRmiiRxD1,
        hal::GpioPinStm& ethernetRmiiTxEn, hal::GpioPinStm& ethernetRmiiTxD0, hal::GpioPinStm& ethernetRmiiTxD1, uint16_t phyAddress)
        : ethernetMdio(ethernetMdio, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetMdc(ethernetMdc, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiRefClk(ethernetRmiiRefClk, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiCrsDv(ethernetRmiiCrsDv, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiRxD0(ethernetRmiiRxD0, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiRxD1(ethernetRmiiRxD1, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiTxEn(ethernetRmiiTxEn, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiTxD0(ethernetRmiiTxD0, hal::PinConfigTypeStm::ethernet, 0)
        , ethernetRmiiTxD1(ethernetRmiiTxD1, hal::PinConfigTypeStm::ethernet, 0)
        , phyAddress(phyAddress)
    {
        EnableClockEthernet(0);

        SetMiiClockRange();
        RunPhy();
    }

    EthernetSmiStm::~EthernetSmiStm()
    {
        DisableClockEthernet(0);
    }

    uint16_t EthernetSmiStm::PhyAddress() const
    {
        return phyAddress;
    }

    void EthernetSmiStm::RunPhy()
    {
        sequencer.Load([this]()
            {
                ResetPhy();
                sequencer.While([]()
                    {
                        return true;
                    });
                DetectLink();
                Delay(std::chrono::milliseconds(200));
                sequencer.EndWhile();
            });
    }

    void EthernetSmiStm::SetMiiClockRange()
    {
       //RT: Not neeeded for H5
    }

    void EthernetSmiStm::ResetPhy()
    {
        sequencer.Execute([this]()
        {
            WritePhyRegister(phyBasicControlRegister, infra::Bit<uint16_t>(phyBcrReset));
        });
        sequencer.DoWhile();
        Delay(std::chrono::milliseconds(1));
        sequencer.Execute([this]()
        {
            phyRegisterValue = ReadPhyRegister(phyBasicControlRegister);
        });
        sequencer.EndDoWhile([this]()
        {
            return infra::IsBitSet(phyRegisterValue, phyBcrReset);
        });
        sequencer.Execute([this]()
        {
            WritePhyRegister(phyBasicControlRegister, infra::Bit<uint16_t>(phyBcrAutoNegotiationEnable));
        });
    }

    void EthernetSmiStm::DetectLink()
    {
        sequencer.Execute([this]()
            {
                uint16_t status = ReadPhyRegister(phyBasicStatusRegister);
                bool newLinkUp = infra::IsBitSet(status, phyBsrLinkUp) && infra::IsBitSet(status, phyBsrAutoNegotiationComplete);

                if (newLinkUp != linkUp)
                {
                    linkUp = newLinkUp;

                    if (linkUp)
                    {
                        uint16_t linkAbility = ReadPhyRegister(phyAutoNegotiationAdvertisement);
                        uint16_t linkPartnerAbility = ReadPhyRegister(phyAutoNegotiationLinkPartnerAbility);
                        LinkSpeed speed;
                        if (infra::IsBitSet(linkPartnerAbility, phyAnlpaFullDuplex100MHz) && infra::IsBitSet(linkAbility, phyAnlpaFullDuplex100MHz))
                            speed = LinkSpeed::fullDuplex100MHz;
                        else if (infra::IsBitSet(linkPartnerAbility, phyAnlpaHalfDuplex100MHz) && infra::IsBitSet(linkAbility, phyAnlpaHalfDuplex100MHz))
                            speed = LinkSpeed::halfDuplex100MHz;
                        else if (infra::IsBitSet(linkPartnerAbility, phyAnlpaFullDuplex10MHz) && infra::IsBitSet(linkAbility, phyAnlpaFullDuplex10MHz))
                            speed = LinkSpeed::fullDuplex10MHz;
                        else
                            speed = LinkSpeed::halfDuplex10MHz;

                        GetObserver().LinkUp(speed);
                    }
                    else
                        GetObserver().LinkDown();
                }
            });
    }

    uint16_t EthernetSmiStm::ReadPhyRegister(uint16_t reg)
    {
        uint32_t tickstart;
        uint32_t tmpreg;
        uint32_t RegValue;

        /* Check for the Busy flag */
        if (READ_BIT(peripheralEthernet[0]->MACMDIOAR, ETH_MACMDIOAR_MB) != (uint32_t)RESET)
        {
            return 0;
        }

        /* Get the  MACMDIOAR value */
        WRITE_REG(tmpreg, peripheralEthernet[0]->MACMDIOAR);

        /* Prepare the MDIO Address Register value
            - Set the PHY device address
            - Set the PHY register address
            - Set the read mode
            - Set the MII Busy bit */

        MODIFY_REG(tmpreg, ETH_MACMDIOAR_PA, (phyAddress << 21));
        MODIFY_REG(tmpreg, ETH_MACMDIOAR_RDA, (reg << 16));
        MODIFY_REG(tmpreg, ETH_MACMDIOAR_MOC, ETH_MACMDIOAR_MOC_RD);
        SET_BIT(tmpreg, ETH_MACMDIOAR_MB);

        /* Write the result value into the MDII Address register */
        WRITE_REG(peripheralEthernet[0]->MACMDIOAR, tmpreg);

        tickstart = HAL_GetTick();

        /* Wait for the Busy flag */
        while (READ_BIT(peripheralEthernet[0]->MACMDIOAR, ETH_MACMDIOAR_MB) > 0U)
        {
            if (((HAL_GetTick() - tickstart) > ETH_MDIO_BUS_TIMEOUT))
            {
                return 0;
            }
        }

        /* Get MACMIIDR value */
        WRITE_REG(RegValue, (uint16_t)peripheralEthernet[0]->MACMDIODR);

        return (uint16_t)RegValue;
    }

    void EthernetSmiStm::WritePhyRegister(uint16_t reg, uint16_t value)
    {
        uint32_t tickstart;
        uint32_t tmpreg;

        /* Check for the Busy flag */
        if (READ_BIT(peripheralEthernet[0]->MACMDIOAR, ETH_MACMDIOAR_MB) != (uint32_t)RESET)
        {
            return;
        }

        /* Get the  MACMDIOAR value */
        WRITE_REG(tmpreg, peripheralEthernet[0]->MACMDIOAR);

        /* Prepare the MDIO Address Register value
            - Set the PHY device address
            - Set the PHY register address
            - Set the write mode
            - Set the MII Busy bit */

        MODIFY_REG(tmpreg, ETH_MACMDIOAR_PA, (phyAddress << 21));
        MODIFY_REG(tmpreg, ETH_MACMDIOAR_RDA, (reg << 16));
        MODIFY_REG(tmpreg, ETH_MACMDIOAR_MOC, ETH_MACMDIOAR_MOC_WR);
        SET_BIT(tmpreg, ETH_MACMDIOAR_MB);

        /* Give the value to the MII data register */
        WRITE_REG(ETH->MACMDIODR, (uint16_t)value);

        /* Write the result value into the MII Address register */
        WRITE_REG(ETH->MACMDIOAR, tmpreg);

        tickstart = HAL_GetTick();

        /* Wait for the Busy flag */
        while (READ_BIT(peripheralEthernet[0]->MACMDIOAR, ETH_MACMDIOAR_MB) > 0U)
        {
            if (((HAL_GetTick() - tickstart) > ETH_MDIO_BUS_TIMEOUT))
            {
                return;
            }
        }
    }

    void EthernetSmiStm::Delay(infra::Duration duration)
    {
        delay = duration;
        sequencer.Step([this]()
            {
                delayTimer.Start(delay, [this]()
                    {
                        sequencer.Continue();
                    });
            });
    }
}

#endif
