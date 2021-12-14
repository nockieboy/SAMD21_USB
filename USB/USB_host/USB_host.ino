/*
 SAMD21G USB HID interface for uCOM GPU

 Handles USB host duties and sending the data
 on to the FPGA via SPI.

 SPI Master must read three bytes from the slave
 to receive all the information on ONE key event,
 so bytes must be read in three.

 Public domain - do with it as you want.
 
 J.Nock/10103 2021
 */

#include <KeyboardController.h> // This is required
#include <SPI.h>

#define DEBUG // Comment this line out to not print debug data on serial bus
#define TEST  // Comment this line out to return keycode data instead of test data

const int slaveAPin = 2; // SS (D2 on SAMD21)
const int buf_size  = 2; // Tx buffer size

// Backup serial connection uses Serial1.  This
// outputs serial on TX/RX pins of the SAMD21G.
#define SerialDebug Serial1

// Initialise USB Controller
USBHost usb;

// Attach keyboard controller to USB
KeyboardController keyboard(usb);

// Initialise variables
uint32_t lastUSBstate     = 0;
uint8_t  cmd_data         = 0; // Received data from Master

#ifdef DEBUG
byte     tx_buf[buf_size] = {0xAA,0x55,0xFF}; // Tx buffer is initialised with test values
#else
byte     tx_buf[buf_size] = {0x00,0x00,0x00}; // Tx buffer is initialised empty
#endif

byte     buf_ptr          = 0; // Points to next tx_buf element to transmit

void printKey();

// This function intercepts key press
void keyPressed() {
#ifdef DEBUG
  SerialDebug.print("\nPressed:  ");
#endif
  printKey();
}

// This function intercepts key release
void keyReleased() {
#ifdef DEBUG
  SerialDebug.print("\nReleased: ");
#endif
  printKey();
}

void printKey() {
  // getModifiers() returns a bits field with the modifiers-keys
  tx_buf[0] = keyboard.getModifiers();
  // getOemKey() returns the OEM-code associated with the key
  tx_buf[1] = keyboard.getOemKey();
  // getKey() returns the ASCII translation of OEM key
  tx_buf[2] = keyboard.getKey();
  // Reset buf_ptr
  buf_ptr = 0;

#ifdef DEBUG
  SerialDebug.print(" key:");
  SerialDebug.print(tx_buf[1]);
  SerialDebug.print(" mod:");
  SerialDebug.print(tx_buf[0]);
  SerialDebug.print(" => ");

  if (tx_buf[0] & LeftCtrl)
    SerialDebug.print("L-Ctrl ");
  if (tx_buf[0] & LeftShift)
    SerialDebug.print("L-Shift ");
  if (tx_buf[0] & Alt)
    SerialDebug.print("Alt ");
  if (tx_buf[0] & LeftCmd)
    SerialDebug.print("L-Cmd ");
  if (tx_buf[0] & RightCtrl)
    SerialDebug.print("R-Ctrl ");
  if (tx_buf[0] & RightShift)
    SerialDebug.print("R-Shift ");
  if (tx_buf[0] & AltGr)
    SerialDebug.print("AltGr ");
  if (tx_buf[0] & RightCmd)
    SerialDebug.print("R-Cmd ");

  // getKey() returns the ASCII translation of OEM key
  // combined with modifiers.
  SerialDebug.write(tx_buf[2]);
  SerialDebug.println();
#endif
}

void setup()
{
#ifdef DEBUG
  // Light the LED to show we're working
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
#endif

  // Set up SPI with SS interrupt
  pinMode(slaveAPin, INPUT_PULLUP);
  attachInterrupt(10, SERCOM4_Handler, FALLING);
  spiSlave_init();

#ifdef DEBUG
  // Send some data via the serial port
  SerialDebug.begin( 115200 );
  SerialDebug.println("\nKeyboard Controller Program started");

  if (usb.Init())
    SerialDebug.println("USB host did not start.");
#endif

  delay( 20 );
}

void loop()
{
  // Process USB tasks - not much else to do
  usb.Task();
  
#ifdef DEBUG
  uint32_t currentUSBstate = usb.getUsbTaskState();
  if (lastUSBstate != currentUSBstate) {
    SerialDebug.print("\nUSB state changed: 0x");
    SerialDebug.print(lastUSBstate, HEX);
    SerialDebug.print(" -> 0x");
    SerialDebug.println(currentUSBstate, HEX);
    switch (currentUSBstate) {
      case USB_ATTACHED_SUBSTATE_SETTLE:
        SerialDebug.print(" Device Attached");
        break;
      case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE:
        SerialDebug.print(" Detached, waiting for Device");
        break;
      case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
        SerialDebug.print(" Resetting Device");
        break;
      case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
        SerialDebug.print(" Reset complete");
        break;
      case USB_STATE_CONFIGURING:
        SerialDebug.print(" USB Configuring");
        break;
      case USB_STATE_RUNNING:
        SerialDebug.print(" USB Running");
        break;
    }
    lastUSBstate = currentUSBstate;
  }
#endif
}

void SERCOM4_handler()
{
  noInterrupts(); // Turn further interrupts off while we handle this one
  
  cmd_data  = (uint8_t)SERCOM4->SPI.DATA.reg;    // Read data register
  uint8_t interrupts = SERCOM4->SPI.INTFLAG.reg; // Read SPI interrupt register

  if (interrupts & (1 << 3)) {        // 8 = 1000 = SSL
    SERCOM4->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select interrupt
  }

  if (interrupts & (1 << 2)) {        // 4 = 0100 = RXC
    SERCOM4->SPI.INTFLAG.bit.RXC = 1; // Clear Receive Complete interrupt
#ifdef DEBUG
    // Print received data for testing to serial debug interface
    SerialDebug.print(" Data Received: ");
    SerialDebug.println(cmd_data, HEX);
#endif
    if (cmd_data == 0x55) { // Get data
      // In theory, CMD_DATA can be ignored other than to reset
      // the buf_ptr to zero.  An attempt to read the data register
      // will result in a DRE interrupt, where the register will
      // be updated with the first byte of the keycode data, if any.
#ifdef DEBUG
      SerialDebug.println(" CMD_GETDATA received.");
#endif
      //SERCOM4->SPI.DATA.reg = tx_buf[buf_ptr];
    } else if (cmd_data == 0xFF) { // Reset
#ifdef DEBUG
      SerialDebug.println(" CMD_RESET received.");
#endif
      tx_buf[0] = 0;
      tx_buf[1] = 0;
      tx_buf[2] = 0;
      buf_ptr   = 0;
      SERCOM4->SPI.DATA.reg = tx_buf[buf_ptr];
    } else if (cmd_data = 0x10 ) { // Get Software Version Number
#ifdef DEBUG
      SerialDebug.println(" CMD_SVN received.");
#endif
      tx_buf[0] = 0x00; // Major version
      tx_buf[1] = 0x01; // Minor version
      tx_buf[2] = 0x08; // Build number
      buf_ptr   = 0;
      SERCOM4->SPI.DATA.reg = tx_buf[buf_ptr];
    }
  }

  if (interrupts & (1 << 1)) { // 2 = 0010 = TXC
    SERCOM4->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }

  if (interrupts & (1 << 0)) // 1 = 0001 = DRE (Data Register Empty)
  {
    SERCOM4->SPI.DATA.reg = tx_buf[buf_ptr]; // Put next byte from tx_buf into SPI register
#ifndef TEST
    tx_buf[buf_ptr] = 0x00; // Clear the tx_buf byte
#endif
    buf_ptr++; // Increment buf_ptr
    if (buf_ptr > buf_size) {
      buf_ptr = 0;
    }
  }
  
  //char _data = data; // data could be a RX'd command to act upon in main loop
  //Serial.print(_data); // print received data
  
  interrupts(); // Turn interrupts back on again ready for the next one
}

void spiSlave_init()
{
  // Configure SERCOM4 SPI pins:
  // - Set PB09 as input  (SS)   - on pin 8  - PAD1
  // - Set PB10 as input  (MOSI) - on pin 19 - PAD2
  // - Set PB11 as input  (SCK)  - on pin 20 - PAD3
  // - Set PA12 as output (MISO) - on pin 21 - PAD0
  // See Table 22-3, p.403 in the datasheet for more information on the PMUXE/PMUXO settings.
  PORT->Group[PORTB].PINCFG[9].bit.PMUXEN  = 0x1; // Enable PMux for SERCOM4 SPI PB09 PIN08 PAD1
  PORT->Group[PORTB].PMUX[4].bit.PMUXO     = 0x3; // SERCOM 4 selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  PORT->Group[PORTB].PINCFG[10].bit.PMUXEN = 0x1; // Enable PMux for SERCOM4 SPI PB10 PIN19 PAD2
  PORT->Group[PORTB].PMUX[5].bit.PMUXE     = 0x3; // SERCOM 4 selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  PORT->Group[PORTB].PINCFG[11].bit.PMUXEN = 0x1; // Enable PMux for SERCOM4 SPI PB11 PIN20 PAD3
  PORT->Group[PORTB].PMUX[5].bit.PMUXO     = 0x3; // SERCOM 4 selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  PORT->Group[PORTA].PINCFG[12].bit.PMUXEN = 0x1; // Enable PMux for SERCOM4 SPI PA12 PIN21 PAD0
  PORT->Group[PORTA].PMUX[6].bit.PMUXE     = 0x3; // SERCOM 4 selected for peripheral use of this pad (0x3 selects peripheral function D: SERCOM-ALT)
  /*
  Explanation:
  PMUXEN stands for Peripheral Multiplexing Enable
  PMUXE stands for Even bits in the Peripheral Multiplexing register
  PMUXO stands for Even bits in the Peripheral Multiplexing register
  The selection of peripheral function A to H is done by writing to the Peripheral Multiplexing
  Odd and Even bits in the Peripheral Multiplexing register (PMUXn.PMUXE/O) in the PORT.
  (Reference: Atmel-42181G-SAM-D21_Datasheet section 6.1 on page 21)
  PB09 corresponds to: PORTB, PMUX[4] Odd
  PB10 corresponds to: PORTB, PMUX[5] Even
  PB11 corresponds to: PORTB, PMUX[5] Odd
  PA12 corresponds to: PORTA, PMUX[6] Even
  In general:
  Px(2n+0/1) corresponds to Portx, PMUX[n] Even=0/Odd=1
  */

  // Disable SPI 1
  SERCOM4->SPI.CTRLA.bit.ENABLE = 0;
  while (SERCOM4->SPI.SYNCBUSY.bit.ENABLE);

  // Reset SPI 1
  SERCOM4->SPI.CTRLA.bit.SWRST = 1;
  while (SERCOM4->SPI.CTRLA.bit.SWRST || SERCOM4->SPI.SYNCBUSY.bit.SWRST);

  // Setting up NVIC
  NVIC_EnableIRQ(SERCOM4_IRQn);
  NVIC_SetPriority(SERCOM4_IRQn, 2);

  // Setting Generic Clock Controller!!!!
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_SERCOM4_CORE) | // Generic Clock 0
                      GCLK_CLKCTRL_GEN_GCLK0 |            // Generic Clock Generator 0 is the source
                      GCLK_CLKCTRL_CLKEN;                 // Enable Generic Clock Generator

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); //Wait for synchronisation

  // Set up SPI Control A Register
  SERCOM4->SPI.CTRLA.bit.DORD     = 0;   // MSB first
  SERCOM4->SPI.CTRLA.bit.CPOL     = 0;   // SCK is low when idle, leading edge is rising edge
  SERCOM4->SPI.CTRLA.bit.CPHA     = 0;   // Data sampled on leading SCK edge and changed on a trailing SCK edge
  SERCOM4->SPI.CTRLA.bit.FORM     = 0x0; // Frame format = SPI
  SERCOM4->SPI.CTRLA.bit.DIPO     = 0x2; // DI=PAD[2] MOSI is used as input (slave mode)
  SERCOM4->SPI.CTRLA.bit.DOPO     = 0x3; // DO=PAD[0], SCK=PAD[3], SS=PAD[1] (Table 26-7, p.494 in datasheet)
  SERCOM4->SPI.CTRLA.bit.MODE     = 0x2; // SPI in Slave mode
  SERCOM4->SPI.CTRLA.bit.IBON     = 0x1; // Buffer Overflow notification
  SERCOM4->SPI.CTRLA.bit.RUNSTDBY = 1;   // Wake on receiver complete

  // Set up SPI control B register
  //SERCOM4->SPI.CTRLB.bit.RXEN = 0x1;    // Enable Receiver
  SERCOM4->SPI.CTRLB.bit.SSDE   = 0x1;    // Slave Select Detection Enabled
  SERCOM4->SPI.CTRLB.bit.CHSIZE = 0;      // Character size 8 Bit
  //SERCOM4->SPI.CTRLB.bit.PLOADEN = 0x1; // Enable Preload Data Register
  //while (SERCOM4->SPI.SYNCBUSY.bit.CTRLB);

  // Set up SPI interrupts
  SERCOM4->SPI.INTENSET.bit.SSL   = 0x1; // Enable Slave Select low interrupt
  SERCOM4->SPI.INTENSET.bit.RXC   = 0x1; // Receive complete interrupt
  SERCOM4->SPI.INTENSET.bit.TXC   = 0x1; // Receive complete interrupt
  SERCOM4->SPI.INTENSET.bit.ERROR = 0x1; // Receive complete interrupt
  SERCOM4->SPI.INTENSET.bit.DRE   = 0x1; // Data Register Empty interrupt
  
  // Init SPI CLK
  //SERCOM4->SPI.BAUD.reg = SERCOM_FREQ_REF / (2*4000000u)-1;
  
  // Enable SPI
  SERCOM4->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM4->SPI.SYNCBUSY.bit.ENABLE);
  SERCOM4->SPI.CTRLB.bit.RXEN   = 0x1;     // Enable Receiver, this is done here due to errate issue
  while (SERCOM4->SPI.SYNCBUSY.bit.CTRLB); // Wait until receiver is enabled
}
