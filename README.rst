Testing driver for st25r100 NFC transceiver on an nRF52840dk.
NCS v2.7.0

# NFCV tag reader

## RFAL Library
Test

- rfalNfcInitialize()
    - rfalAnalogConfigInitialize()
    - rfalInitialize()
    - st25r200Initialize()
        - Direct command to set default (0x60)
        - Check chip ID
        - st25r200OscOn()
        - Clear and enable interrupts
        - Set en bit in operation register (0x00)
        - Wait for oscillator interrupt
        - st25r200WaitAgd()
            - Check that agd_ok bit is set in display register 1 (0x0F)
        - st25r200TxRxOff()
        - Clear tx_en and rx_en bits in operation register (0x00)
        - Disable interrupts
        - Clear interrupts
    - rfalSetAnalogConfig((RFAL_ANALOG_CONFIG_TECH_CHIP | RFAL_ANALOG_CONFIG_CHIP_INIT))
        - Set miso_pd1 and miso_pd2 bits in general register (0x01)
        - Set am_en bit in operation register (0x00)
        - Set reg_am and am_mod bits in tx modulation register (0x04)
        - Set d_res bits in tx driver register to 0
        - Set en_subc bit in correlator 5 register (0x0D)
        - Set afe_gain_td bits to 4 [34dB] in analog register 2 (0x07)
    - rfalCalibrate() regulators
        - st25r200AdjustRegulators()
        - Clear reg_s bit in general register (0x01)
        - Execute adjust regulators direct command (0x68)
- rfalNfcDiscover(&[discParam](#discParam))
    - Initializes and validates [rfalNfc struct](#rfalNfc)
    - rfalNfcDeactivation()
- rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE)
    - rfalFieldOff()
    - Clears register bits tx_en and rx_en in the configuration register (@0x00)

## RFAL Library References

### RFAL Analog Configuration Table Structure
