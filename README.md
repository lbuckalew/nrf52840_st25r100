Testing driver for st25r100 NFC transceiver on an nRF52840dk.
NCS v2.7.0

# NFCV tag reader

## RFAL Library Tracing

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

## RFAL References

### Configuration Packet Structure
```
/*
 ******************************************************************************
 * The Analog Configuration is structured as following
 * +---------+-----------------------+-----------------------------+
 * | ModeID  | Num RVM configuration | RVM (Register, Value, Mask) |
 * | (16bit) | (8bit)                | (24bit)                     |
 * +---------+-----------------------+-----------------------------+
 *
 * The Mode ID coding for different use cases is described below
 * 
 * 1. ModeID coding for NFC technologies (not chip specific)
 * +----------------------------------------------------------------------+
 * | 15  | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +----------------------------------------------------------------------+
 * | P/L | TECH != CHIP                   | BR            | DIR           |
 * +----------------------------------------------------------------------+
 * 
 * 2. ModeID coding for chip specific modes and events
 * +----------------------------------------------------------------------+
 * | 15  | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +----------------------------------------------------------------------+
 * | P/L | TECH == CHIP                   | CHIP_SPECIFIC                 |
 * +----------------------------------------------------------------------+
 * 
 * 3. Special ModeID coding for Direction == DPO
 * +----------------------------------------------------------------------+
 * | 15  | 14 | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
 * +----------------------------------------------------------------------+
 * | P/L | DPO_LVL | TECH*                | BR            | DIR == DPO    |
 * +----------------------------------------------------------------------+
 *            ^
 *            | 
 *            +----- reuse of TECH_RFU bits as DPO level indicator
 ******************************************************************************
 */
```

### discParam

```c
/*! Discovery parameters                                                                                                             */
typedef struct{
    rfalComplianceMode     compMode;                         /*!< Compliancy mode to be used                                         */
    uint16_t               techs2Find;                       /*!< Technologies to search for                                         */
    uint16_t               techs2Bail;                       /*!< Bail-out after certain NFC technologies                            */
    uint16_t               totalDuration;                    /*!< Duration of a whole Poll + Listen cycle        NCI 2.1 Table 46    */
    uint8_t                devLimit;                         /*!< Max number of devices                      Activity 2.1  Table 11  */
    rfalBitRate            maxBR;                            /*!< Max Bit rate to be used                        NCI 2.1  Table 28   */
    rfalBitRate            nfcfBR;                           /*!< Bit rate to poll for NFC-F                     NCI 2.1  Table 27   */
    uint8_t                nfcid3[RFAL_NFCDEP_NFCID3_LEN];   /*!< NFCID3 to be used on the ATR_REQ/ATR_RES                           */
    uint8_t                GB[RFAL_NFCDEP_GB_MAX_LEN];       /*!< General bytes to be used on the ATR-REQ        NCI 2.1  Table 29   */
    uint8_t                GBLen;                            /*!< Length of the General Bytes                    NCI 2.1  Table 29   */
    rfalBitRate            ap2pBR;                           /*!< Bit rate to poll for AP2P                      NCI 2.1  Table 31   */
    bool                   p2pNfcaPrio;                      /*!< NFC-A P2P (true) or ISO14443-4/T4T (false) priority                */
    rfalNfcPropCallbacks   propNfc;                          /*!< Proprietary Technlogy callbacks                                    */
    rfalIsoDepFSxI         isoDepFS;                         /*!< ISO-DEP Poller announced maximum frame size   Digital 2.2 Table 60 */
    uint8_t                nfcDepLR;                         /*!< NFC-DEP Poller & Listener maximum frame size  Digital 2.2 Table 90 */
    rfalLmConfPA           lmConfigPA;                       /*!< Configuration for Passive Listen mode NFC-A                        */
    rfalLmConfPF           lmConfigPF;                       /*!< Configuration for Passive Listen mode NFC-A                        */
    void                   (*notifyCb)( rfalNfcState st );   /*!< Callback to Notify upper layer                                     */
    bool                   wakeupEnabled;                    /*!< Enable Wake-Up mode before polling                                 */
    bool                   wakeupConfigDefault;              /*!< Wake-Up mode default configuration                                 */
    rfalWakeUpConfig       wakeupConfig;                     /*!< Wake-Up mode configuration                                         */
    bool                   wakeupPollBefore;                 /*!< Flag to Poll wakeupNPolls times before entering Wake-up            */
    uint16_t               wakeupNPolls;                     /*!< Number of polling cycles before|after entering Wake-up             */
}rfalNfcDiscoverParam;
```

### rfalNfc

```c
/*! RFAL NFC instance                                                                                */
typedef struct{
    rfalNfcState            state;              /*!< Main state                                      */
    uint16_t                techsFound;         /*!< Technologies found bitmask                      */
    uint16_t                techs2do;           /*!< Technologies still to be performed              */
    uint16_t                techDctCnt;         /*!< Technologies detection counter (before WU)      */
    rfalBitRate             ap2pBR;             /*!< Bit rate to poll for AP2P                       */
    uint8_t                 selDevIdx;          /*!< Selected device index                           */
    rfalNfcDevice           *activeDev;         /*!< Active device pointer                           */
    rfalNfcDiscoverParam    disc;               /*!< Discovery parameters                            */
    rfalNfcDevice           devList[RFAL_NFC_MAX_DEVICES];   /*!< Location of device list            */
    uint8_t                 devCnt;             /*!< Decices found counter                           */
    uint32_t                discTmr;            /*!< Discovery Total duration timer                  */
    ReturnCode              dataExErr;          /*!< Last Data Exchange error                        */
    rfalNfcDeactivateType   deactType;          /*!< Deactivation type                               */
    bool                    isRxChaining;       /*!< Flag indicating Other device is chaining        */
    uint32_t                lmMask;             /*!< Listen Mode mask                                */
    bool                    isFieldOn;          /*!< Flag indicating Fieldon for Passive Poll        */
    bool                    isTechInit;         /*!< Flag indicating technology has been set         */
    bool                    isOperOngoing;      /*!< Flag indicating operation is ongoing            */
    bool                    isDeactivating;     /*!< Flag indicating deactivation is ongoing         */
    rfalNfcaSensRes         sensRes;            /*!< SENS_RES during card detection and activation   */
    rfalNfcbSensbRes        sensbRes;           /*!< SENSB_RES during card detection and activation  */
    uint8_t                 sensbResLen;        /*!< SENSB_RES length                                */
    rfalNfcBuffer           txBuf;              /*!< Tx buffer for Data Exchange                     */
    rfalNfcBuffer           rxBuf;              /*!< Rx buffer for Data Exchange                     */
    uint16_t                rxLen;              /*!< Length of received data on Data Exchange        */
#if RFAL_FEATURE_NFC_DEP || RFAL_FEATURE_ISO_DEP
    rfalNfcTmpBuffer        tmpBuf;             /*!< Tmp buffer for Data Exchange                    */
#endif /* RFAL_FEATURE_NFC_DEP || RFAL_FEATURE_ISO_DEP */

}rfalNfc;
```