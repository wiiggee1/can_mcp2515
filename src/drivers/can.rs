//! MCP2515 driver v2 - for writing CAN frame messages over the CAN bus.
//!
//! ## Note
//!
//! This driver is an updated version of the first driver.
//! The driver is still under progress and might change slightly over time.
#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]

use core::{borrow::Borrow, str};

use super::message::CanMessage;
use digital::ErrorKind;
use embedded_can::{blocking::Can, Error, Frame, Id, StandardId};
use embedded_hal::{
    digital,
    digital::{InputPin, OutputPin},
    spi::SpiBus,
};

/// The `MCP2515` driver struct.
pub struct Mcp2515Driver<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> {
    /// Uses the `SpiBus` trait from `embedded_hal::spi::SpiBus`
    ///
    /// This field, can write, read over the Spi bus.
    pub spi: SPI,
    /// Chip-Select Pin, for driving the pin from high to low, during SPI
    /// transactions to the MCP2515.  
    pub cs: PIN,
    /// Interrupt input pin, that would generate a GPIOTE event on falling edge.
    /// Going from high to low.
    pub interrupt_pin: PININT,
    /// This is MCP2515 specific settings, for applying settings such as:
    /// - CANCTRL register bit settings.
    /// - MCP clock speed.
    /// - CAN bitrate.
    /// - Enabling of interrupts.
    /// - Receive buffer mode (e.g., turn mask/filters off, Only `StandardID`,
    ///   etc...).
    pub can_settings: Mcp2515Settings,

    /// Array were each index represent which TX buffer that is currently
    /// busy or pending.
    pub busy_tx: [bool; 3],

    /// This is for reading the specific RX buffer depending on the given
    /// interrupt received.
    pub active_rx: (Option<RXBN>, bool),
}

///Instruction commands specific to the MCP2515 via SPI.
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum InstructionCommand {
    /// Resets the internal registers to the default state.
    Reset = 0xC0,
    /// Reads data from the register beginning at selected address.
    Read = 0x03,
    /// Writes data to the register beginning at the selected address.
    Write = 0x02,
    /// Quick polling command that reads several status bits for transmit and
    /// receive functions.
    Status = 0xA0,
    /// Allow the user to set or clear individual bits in a particular register.
    Modify = 0x05,
    /// Loading a transmit buffer, this instruction reduce the overheaad of a
    /// normal WRITE command.
    LoadTX = 0x40,
    /// Request-To-Send - begin message transmission sequence for any of the
    /// transmit buffers.
    RTS = 0x80,
    /// Reading the specific receive buffer 0.
    ReadRxb0 = 0x90,
    /// Reading the specific receive buffer 1.
    ReadRxb1 = 0x94,
    /// Quick polling command that indicates filter match and message type of
    /// receive message.
    RxStatus = 0xB0,
}

/// The MCP2515 modes of operation.
///
/// REQOP: Request Operation Mode bits <2:0>
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperationTypes {
    /// 100 = Set Configuration mode
    Configuration = 0b100,
    /// 000 = Set Normal Operation mode
    Normal = 0b000,
    /// 001 = Set Sleep mode
    Sleep = 0b001,
    /// 011 = Set Listen-only mode
    ListenOnly = 0b011,
    /// 010 = Set Loopback mode
    Loopback = 0b010,
}

/// Start address for the Transmit buffer N.
#[derive(Clone, Copy)]
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum TXBN {
    TXB0 = 0x30,
    TXB1 = 0x40,
    TXB2 = 0x50,
}

impl TXBN {
    /// Return the associated array index for the specific `TXBN` enum value.
    const fn idx(self) -> usize {
        match self {
            Self::TXB0 => 0,
            Self::TXB1 => 1,
            Self::TXB2 => 2,
        }
    }
}

/// Denotes the valid RX buffers.
#[derive(Clone, Copy)]
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
pub enum RXBN {
    RXB0 = 0x60,
    RXB1 = 0x70,
}

/// Settings related to register RXBnCTRL.RXM bits.
/// For setting up, receive buffer operating mode.
#[derive(Clone, Copy)]
pub enum ReceiveBufferMode {
    FilterOffReceiveAny,
    OnlyExtendedId,
    OnlyStandardId,
    ReceiveAny,
}

/// Receive Buffer Mask N.
#[derive(Clone, Copy)]
#[allow(clippy::upper_case_acronyms)]
enum RXMN {
    RXM0,
    RXM1,
}

/// Receive Buffer Filter N.
#[derive(Clone, Copy)]
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum RXFN {
    RXF0 = 0b0001_0000,
    RXF1,
    RXF2,
    RXF3,
    RXF4,
    RXF5,
}

/// Bits for setting the clock prescale.
#[derive(Clone, Copy)]
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
pub enum CLKPRE {
    /// System Clock/1
    DIV1 = 0b000,
    ///System Clock/2
    DIV2 = 0b001,
    /// System Clock/4
    DIV4 = 0b010,
    /// System Clock/8
    DIV8 = 0b011,
}

/// MCP2515 module register upper address.
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum MCP2515Register {
    /// CAN Status Register.
    CANSTAT = 0x0E,
    /// CAN Control Register.
    CANCTRL = 0x0F,
    /// CAN Interrupt Enable Register.
    CANINTE = 0x2B,
    /// CAN Interrupt Flag Register.
    CANINTF = 0x2C,

    /// `RXnBF` PIN control and status register.
    BFPCTRL = 0x0C,
    /// `TXnRTS` PIN control and status register.
    TXRTSCTRL = 0x0D,
    /// Transmit Error Counter Register.
    TEC = 0x1C,
    /// Receive Error Counter Register.
    REC = 0x1D,
    /// Configuration register 1.
    CNF1 = 0x2A,
    /// Configuration register 2.
    CNF2 = 0x29,
    /// Configuration register 3
    CNF3 = 0x28,
    /// Error Flag Register.
    EFLG = 0x2D,

    /// Transmit buffer 0 Control Register.
    TXB0CTRL = 0x30,
    /// Transmit buffer 1 Control Register.
    TXB1CTRL = 0x40,
    /// Transmit buffer 2 Control Register.
    TXB2CTRL = 0x50,

    /// Receive buffer 0 control register.
    RXB0CTRL = 0x06,
    /// Receive buffer 1 control register.
    RXB1CTRL = 0x07,

    /// Mask 0 - Standard Identifier Register High.
    RXM0SIDH = 0x20,
    /// Mask 0 - Standard Identifier Register Low.
    RXM0SIDL = 0x21,
    /// Mask 1 - Standard Identifier Register High.
    RXM1SIDH = 0x24,
    /// Mask 1 - Standard Identifier Register Low.
    RXM1SIDL = 0x25,

    /// Filter 0 - Standard Identifier Register High.
    RXF0SIDH = 0x00,
    /// Filter 0 - Standard Identifier Register Low.
    RXF0SIDL = 0x01,

    /// Filter 1 - Standard Identifier Register High.
    RXF1SIDH = 0x04,
    /// Filter 1 - Standard Identifier Register Low.
    RXF1SIDL = 0x05,

    /// Filter 2 - Standard Identifier Register High.
    RXF2SIDH = 0x08,
    /// Filter 2 - Standard Identifier Register Low.
    RXF2SIDL = 0x09,

    /// Filter 3 - Standard Identifier Register High.
    RXF3SIDH = 0x10,
    /// Filter 3 - Standard Identifier Register Low.
    RXF3SIDL = 0x11,
    /// Filter 4 - Standard Identifier Register High.
    RXF4SIDH = 0x14,
    /// Filter 4 - Standard Identifier Register Low.
    RXF4SIDL = 0x15,
    /// Filter 5 - Standard Identifier Register High.
    RXF5SIDH = 0x18,
    /// Filter 5 - Standard Identifier Register Low.
    RXF5SIDL = 0x19,
}

/// MCP2515 driver specific error types.
#[derive(Debug)]
pub enum DriverError {
    /// Error related to the SPI bus
    SpiError,

    /// CAN Frame related error.
    FrameError,
}

/// Associated registers to a specific TX/RX buffer. Going from high to low
/// addresses offsets.
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum TxRxbn {
    /// TXBNCTRL/RXBNCTRL
    CTRL = 0x0,
    /// TXBNSIDH/RXBNSIDH
    SIDH = 0x1,
    /// TXBNSIDL/RXBNSIDL
    SIDL = 0x2,
    /// TXBNEID8/RXBNEID8 - Extended ID
    EID8 = 0x3,
    /// TXBNEID0/RXBNEID0 - Extended ID
    EID0 = 0x4,
    /// TXBNDLC/RXBNDLC - Data Length Code
    DLC = 0x5,
    /// TXBND0/RXBND0 - Data byte 0
    D0 = 0x6,
    /// TXBND1/RXBND1 - Data byte 1
    D1 = 0x7,
    /// TXBND2/RXBND2 - Data byte 2
    D2 = 0x8,
    /// TXBND3/RXBND3 - Data byte 3
    D3 = 0x9,
    /// TXBND4/RXBND4 - Data byte 4
    D4 = 0x10,
    /// TXBND5/RXBND5 - Data byte 5
    D5 = 0x11,
    /// TXBND6/RXBND6 - Data byte 6
    D6 = 0x12,
    /// TXBND7/RXBND7 - Data byte 7
    D7 = 0x13,
}

/// MCP2515 CANCTRL register setting bits.  
#[derive(Clone, Copy)]
pub struct SettingsCanCtrl {
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool,  // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

/// MCP2515 register CANINTE.
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(clippy::module_name_repetitions)]
pub enum CanInte {
    /// Message Error Interrupt bit - during message reception/transmission.
    MERRE = 0b1000_0000,

    /// Wake-up Interrupt bit - on CAN bus activity.
    WAKIE = 0b0100_0000,

    /// Error Interrupt bit - on EFLG error condition change.
    ERRIE = 0b0010_0000,

    /// Transmit buffer 2 empty interrupt bit - when TXB2 becomes empty (sent
    /// message).
    TX2IE = 0b0001_0000,

    /// Transmit buffer 1 empty interrupt bit - when TXB1 becomes empty (sent
    /// message).
    TX1IE = 0b0000_1000,

    /// Transmit buffer 0 empty interrupt bit - when TXB0 becomes empty (sent
    /// message).
    TX0IE = 0b0000_0100,

    /// Receive buffer 1 full interrupt bit - message received in RXB1.
    RX1IE = 0b0000_0010,

    /// Receive buffer 0 full interrupt bit - message received in RXB0.
    RX0IE = 0b0000_0001,
}

/// MCP2515 register CANSTAT.ICOD - Interrupt Flag Code bits.
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum InterruptFlagCode {
    NoInterrupt = 0b000,
    ErrorInterrupt = 0b001,
    WakeUpInterrupt = 0b010,
    TXB0Interrupt = 0b011,
    TXB1Interrupt = 0b100,
    TXB2Interrupt = 0b101,
    RXB0Interrupt = 0b110,
    RXB1Interrupt = 0b111,
}

/// MCP2515 Clock speed in Mega-hertz.
#[derive(Clone, Copy)]
pub enum McpClock {
    /// 16 `MHz`.
    MCP16,

    /// 8 `MHz`.
    MCP8,
}

/// Transmission rate in kBPS.
#[derive(Clone, Copy)]
pub enum Bitrate {
    /// 125 kBPS
    CAN125,
}

#[derive(Clone, Copy)]
/// Acceptable frame ID that the RX0 or RX1 buffer should accept on the CAN bus.
/// This would determine if a message should be loaded into either of the
/// receive buffers.
///
/// - `rx_mask` - the mask determine which bits in the identifier that should be
///   examined with the filter.
///
/// - `acceptance filter` - are compared with identifier fields of the message.
pub struct AcceptanceFilterMask {
    rx_mask: u16,
    acceptance_filter: u16,
}

/// Struct for necessary settings for setting up the MCP2515 CAN module.
#[derive(Clone, Copy)]
pub struct Mcp2515Settings {
    canctrl: SettingsCanCtrl,
    mcp_clk: McpClock, // Clock Frequency for MCP2515.
    can_bitrate: Bitrate,
    pub interrupts: u8,
    rxm_mode: ReceiveBufferMode,
    rx0_filtermask: AcceptanceFilterMask,
    rx1_filtermask: AcceptanceFilterMask,
}

impl AcceptanceFilterMask {
    const DEFAULT_FILTER_MASK: u16 = 0u16;

    /// Creates a neew Acceptance Filter and mask, for constraining acceptable
    /// frame IDs, that should be accepted/ignored.
    ///
    /// # Panics
    ///
    /// Whenever the 11-bit `StandardId` is out of range whenever > 0x7FF.
    #[must_use]
    pub fn new(mask: u16, filter: u16) -> Self {
        let acceptance_id = (StandardId::new(filter).unwrap().as_raw() << 5);

        Self {
            rx_mask: mask,
            acceptance_filter: acceptance_id,
        }
    }
}

impl Default for AcceptanceFilterMask {
    fn default() -> Self {
        Self { rx_mask: Self::DEFAULT_FILTER_MASK, acceptance_filter: Self::DEFAULT_FILTER_MASK }
    }
}

impl SettingsCanCtrl {
    /// Creates a new MCP2515 module specific settings instance struct.
    ///
    /// ## NOTE
    ///
    /// This will set:
    /// - Operation mode (Configuration, Normal, Loopback, etc...).
    /// - 'Abort All Pending Transmission' (ABAT), 'One-Shot Mode' (transmit one
    ///   time only).
    /// - CLKOUT pin enable/disable (CLKEN).
    /// - CLKOUT Pin Prescler (CLKPRE).
    #[must_use]
    pub const fn new(
        mode: OperationTypes,
        clken: bool,
        clkpre: CLKPRE,
        abat: bool,
        osm: bool,
    ) -> Self {
        Self {
            mode,
            clken,
            clkpre,
            abat,
            osm,
        }
    }
}

impl Default for Mcp2515Settings {
    fn default() -> Self {
        let canctrl_settings = SettingsCanCtrl::new(
            OperationTypes::Configuration,
            true,
            CLKPRE::DIV1,
            false,
            true,
        );
        Self {
            canctrl: canctrl_settings,
            mcp_clk: McpClock::MCP8,
            can_bitrate: Bitrate::CAN125,
            interrupts: 0u8,
            rxm_mode: ReceiveBufferMode::FilterOffReceiveAny,
            rx0_filtermask: AcceptanceFilterMask::default(),
            rx1_filtermask: AcceptanceFilterMask::default(),
        }
    }
}

impl Mcp2515Settings {
    pub const DEFAULT_FILTER_MASK: u16 = 0u16;

    /// Creates a new `MCP2515Settings` instance with specified settings.
    #[must_use]
    pub const fn new(
        canctrl: SettingsCanCtrl,
        mcp_clk: McpClock,
        can_bitrate: Bitrate,
        interrupts: u8,
        rxm_mode: ReceiveBufferMode,
        rx0_filtermask: AcceptanceFilterMask,
        rx1_filtermask: AcceptanceFilterMask,
    ) -> Self {
        Self {
            canctrl,
            mcp_clk,
            can_bitrate,
            interrupts,
            rxm_mode,
            rx0_filtermask,
            rx1_filtermask,
        }
    }

    /// Enable interrupt in the MCP2515.CANINTE register.
    /// This works by creating an interrupt mask by setting the
    /// associated CANINTE bit position associated with the specific interrupt.
    pub const fn enable_interrupt(mut self, interrupt_type: CanInte) -> Self {
        self.interrupts |= interrupt_type as u8;
        self
    }

    /// Enables interrupt(s) in the MCP2515.CANINTE register.
    /// This works by creating an interrupt mask by setting the
    /// associated CANINTE bit(s) associated with the specific interrupt.
    pub fn enable_interrupts(mut self, interrupt_types: &[CanInte]) -> Self {
        interrupt_types
            .iter()
            .for_each(|el| self.interrupts |= *el as u8);
        self
    }

    /// This is for setting the filtering on the RX0 buffer.
    /// Default: Accepting everything.
    pub fn filter_b0(mut self, mask: u16, filter: u16) -> Self {
        self.rx0_filtermask = AcceptanceFilterMask::new(mask, filter);
        self
    }

    /// This is for setting the filtering on the RX1 buffer.
    /// Default: Accepting everything.
    pub fn filter_b1(mut self, mask: u16, filter: u16) -> Self {
        self.rx1_filtermask = AcceptanceFilterMask::new(mask, filter);
        self
    }

}

impl TryFrom<u8> for InterruptFlagCode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(Self::NoInterrupt),
            0b001 => Ok(Self::ErrorInterrupt),
            0b010 => Ok(Self::WakeUpInterrupt),
            0b011 => Ok(Self::TXB0Interrupt),
            0b100 => Ok(Self::TXB1Interrupt),
            0b101 => Ok(Self::TXB2Interrupt),
            0b110 => Ok(Self::RXB0Interrupt),
            0b111 => Ok(Self::RXB1Interrupt),
            _ => Err(()),
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug)]
#[allow(clippy::upper_case_acronyms)]
enum EFLG {
    /// Receive buffer 1 overflow flag.  
    RX1OVR = 7,

    /// Receive buffer 0 overflow flag.  
    RX0OVR = 6,

    /// Bus-Off Error Flag bit (TEC = 255)
    TXBO = 5,

    /// Transmit Error-Passive Flag bit (TEC >= 128)
    TXEP = 4,

    /// Receive Error-Passive Flag bit (REX >= 128)
    RXEP = 3,

    /// Transmit Error Warning Flag bit (TEC >= 96)
    TXWAR = 2,

    /// Receive Error Warning Flag bit (REC >= 96)
    RXWAR = 1,

    /// Error Warning Flag bit (TXWAR or RXWAR = 1)
    EWARN = 0,

    /// Just for debugging purpose. No known EFLG type.
    UNKNOWN = 0xFF,
}

#[derive(Debug)]
/// Specific Can Error types related to `MCP2515` module. Still under progress!
pub enum Mcp2515Error {
    /// Whenever a Transmission error occurs.
    TransmissionError,

    ///Receive buffer 0 overflow error.  
    RX0Overflow,

    /// Receive buffer 1 overflow error.
    RX1Overflow,

    /// Message error interrupt, when a can frame is faulty.
    MessageErrorInterrupt,

    /// Can module receive error.
    ReceiveError,

    /// Custom error type for whenever decoding a can frame isn't successful.
    DecodeError,

    /// The underlying spi abstraction returned an error.
    SPIError,
}

impl EFLG {
    /// Return the appropriate EFLG type based on provided u8 value.
    fn from_u8(value: u8) -> Self {
        match value {
            0b000 => Self::EWARN,
            0b001 => Self::RXWAR,
            0b010 => Self::TXWAR,
            0b011 => Self::RXEP,
            0b100 => Self::TXEP,
            0b101 => Self::TXBO,
            0b110 => Self::RX0OVR,
            0b111 => Self::RX1OVR,
            _ => unreachable!(),
        }
    }
}

impl embedded_can::Error for Mcp2515Error {
    /// Returns the `ErrorKind` (CAN error) types.  
    #[allow(clippy::match_same_arms)]
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            Self::TransmissionError => embedded_can::ErrorKind::Other,
            Self::RX0Overflow => embedded_can::ErrorKind::Overrun,
            Self::RX1Overflow => embedded_can::ErrorKind::Overrun,
            Self::MessageErrorInterrupt => embedded_can::ErrorKind::Other,
            Self::ReceiveError => embedded_can::ErrorKind::Other,
            Self::DecodeError => embedded_can::ErrorKind::Other,
            Self::SPIError => embedded_can::ErrorKind::Other,
        }
    }
}

impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> Can
    for Mcp2515Driver<SPI, PIN, PININT>
{
    type Error = Mcp2515Error;
    type Frame = CanMessage;

    /// Calls the driver `transmit_can` method for transmitting data to the TX
    /// buffer.
    ///
    /// ## NOTE
    ///
    /// This method is required for the `embedded_can::blocking::Can` trait.  
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        //self.read_status();
        self.transmit_can(frame); // This is the custom driver logic for transmitting on the CAN bus.
        Ok(())
    }

    /// Calls the driver `receive_can` method for reading appropriate RX buffer.
    /// ## NOTE
    ///
    /// This method is required for the `embedded_can::blocking::Can` trait.
    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        let frame = self.receive_can();
        Ok(frame)
    }
}

impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin>
    Mcp2515Driver<SPI, PIN, PININT>
{
    /// Initialize and creates a new driver instance. Based on provided
    /// `CanSettings` argument.
    ///
    /// ## PARAM:
    ///
    /// - SPI: `SpiBus` trait for exclusive ownership - SPI bus pins.
    /// - PIN: `embedded_hal::digital::OutputPin` - Chipselect (CS) pin.
    /// - PININT: `embedded_hal::digital::InputPin` - for MCP2515 INT signals.
    pub fn init(spi: SPI, cs: PIN, interrupt_pin: PININT, can_settings: Mcp2515Settings) -> Self {
        let initial_settings = can_settings.borrow();
        let mut driver = Self {
            spi,
            cs,
            interrupt_pin,
            can_settings,
            busy_tx: [false, false, false],
            active_rx: (None, false),
        };

        driver.reset_instruction();

        driver
            .setup_configuration(initial_settings) // Setup CANCTRL register.
            .setup_bitrate() // Setup bitrate and clk freq, in registers: CNF1, CNF2, & CNF3.
            .setup_interrupt(initial_settings) // Setup interrupts in CANINTE register.
            .tx_pending(false) // CLEAR the TXREQ bits indicating the tx buffer is not pending before writing.
            .set_rxm_mode(initial_settings.rxm_mode)
            .filter_message_id(
                RXBN::RXB0,
                initial_settings.rx0_filtermask.rx_mask,
                initial_settings.rx0_filtermask.acceptance_filter,
            )
            .filter_message_id(
                RXBN::RXB1,
                initial_settings.rx1_filtermask.rx_mask,
                initial_settings.rx1_filtermask.acceptance_filter,
            );


        driver.activate_canbus();

        if (driver.can_settings.canctrl.mode != OperationTypes::Normal) {
            //defmt::panic!("Operational Mode, has to be in Normal Mode, before using CAN Bus!");
        }

        driver
    }

    /// SPI read.
    fn read(&mut self, data: &mut [u8]) {
        self.spi.read(data);
    }

    pub fn read_caninte(&mut self) -> Result<u8, SPI::Error> {
        self.read_register(MCP2515Register::CANINTE, 0x0)
        
    }

    /// SPI write.
    fn write(&mut self, data: &[u8]) {
        self.spi.write(data);
    }

    /// Perform spi transfer by writing to MOSI and received on MISO.
    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) {
        self.cs.set_low();
        self.spi.transfer(read, write);
        self.cs.set_high();
    }

    /// reading a MCP2515 register address + register offset (optional).
    ///
    /// #NOTE
    ///
    /// Perform spi transfer by writing to MOSI and received on MISO.
    fn read_register(&mut self, reg: MCP2515Register, reg_offset: u8) -> Result<u8, SPI::Error> {
        const DONTCARES: u8 = 0u8;
        let mut read_buf: [u8; 3] = [0; 3];
        let register: u8 = reg as u8 + reg_offset;
        let mut read_msg: [u8; 3] = [InstructionCommand::Read as u8, register, DONTCARES];

        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &read_msg)?;
        self.cs.set_high();

        Ok(read_buf[2])
    }

    /// Write u8 data, to a specific MCP2515 register address.
    ///
    /// ## NOTE
    ///
    /// Expected format: [u8; N] --> [Instruction byte, Address byte, Data
    /// byte]. For writing to a specific register
    fn write_register(&mut self, reg: MCP2515Register, data: u8) -> Result<(), SPI::Error> {
        let reg_address: u8 = reg as u8;
        let mut byte_msg: [u8; 3] = [InstructionCommand::Write as u8, reg_address, data];

        self.cs.set_low();
        self.spi.write(&byte_msg)?;
        self.cs.set_high();
        Ok(())
    }

    /// Changing settings, is mostly used for changing the operating mode of the
    /// MCP2515. E.g., changing from 'Configuration' to 'Normal' mode.
    fn change_settings(&mut self, settings: Mcp2515Settings) {
        self.can_settings.canctrl = settings.canctrl;
        let bitmask_canctrl = self.get_canctrl_mask(self.can_settings.canctrl);

        self.write_register(MCP2515Register::CANCTRL, bitmask_canctrl);
    }

    /// This is just for debugging and testing the SPI, for testing read and
    /// write for MCP2515.
    fn transfer_test(&mut self, register: MCP2515Register) {
        let mut read_buf: [u8; 3] = [0; 3];
        let write_buf: [u8; 3] = [InstructionCommand::Read as u8, register as u8, 0x00];
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &write_buf);
        self.cs.set_high();
        //defmt::info!("Sent: {:08b}, Received: {:08b}", write_buf, read_buf);
    }

    /// Loopback test, that sets the mode to loopback, and LOAD TX and READ RX
    /// buffer.
    /// -------------------------------------------------------------------------*
    ///
    /// (1). Change mode by writing to CANCTRL register with the settings
    /// bitmask. (2). Create a new CAN frame and convert to byte array.
    /// (3). Perform 'Load TX buffer' instruction, loading the CAN byte frame.
    /// (4). Initiate transmission by setting the TXREQ bit (`TXBnCTRL`(3))
    /// for each buffer. (5). Poll the target RX buffer, checking if message
    /// has been received. Then read RX!
    /// -------------------------------------------------------------------------*    
    ///
    /// Example format of byte message: ...
    /// ... let `byte_msg`: [u8; 3] = [`InstructionCommand::Write` as u8,
    /// `MCP2515Register::TXB0CTRL` as u8, data];
    pub fn loopback_test(&mut self, can_msg: CanMessage) {
        //defmt::info!("Loopback Test:");
        let mut can_settings = Mcp2515Settings::default();
        let canctrl_settings =
            SettingsCanCtrl::new(OperationTypes::Loopback, false, CLKPRE::DIV1, false, false);
        can_settings.canctrl = canctrl_settings;

        self.change_settings(can_settings);
        self.read_status();
        self.load_tx_buffer(TXBN::TXB0, can_msg);
    }

    /// The `blocking_rx` method was for reading the RX buffers for a loopback
    /// test without interrupts.
    fn blocking_rx(&mut self, rx: RXBN) {
        loop {
            let rx_status = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
            match rx {
                RXBN::RXB0 => {
                    if (rx_status & (1 << 0) != 0) {
                        let received_frame = self.read_rx_buffer(RXBN::RXB0).unwrap();
                        break;
                    }
                }
                RXBN::RXB1 => {
                    if (rx_status & (1 << 1) != 0) {
                        let received_frame = self.read_rx_buffer(RXBN::RXB1).unwrap();
                        break;
                    }
                }
            }
        }
    }

    /// Method for applying the CAN controller settings by getting the bitmask.
    /// Format for setting bit: N = N | (1<<k).
    pub fn get_canctrl_mask(&mut self, canctrl_settings: SettingsCanCtrl) -> u8 {
        let mut canctrl_byte = 0u8; // (1): data_byte |= (reg_value << reg_pos)
        let mode_bits: u8 = (canctrl_settings.mode as u8) << 5; //REQOP[2:0] (bits 7-5)
        let prescale: u8 = canctrl_settings.clkpre as u8; //CLKPRE[1:0] (bits 1-0)
        let clk_enabled = canctrl_settings.clken;

        canctrl_byte |= mode_bits; //(2)

        if (canctrl_settings.abat) {
            canctrl_byte |= 1 << 4;
        }

        if (canctrl_settings.osm) {
            canctrl_byte |= 1 << 3;
        }

        if (clk_enabled) {
            canctrl_byte |= 1 << 2;
        } else {
            canctrl_byte &= !(1 << 2);
        }

        canctrl_byte |= prescale; //(3)

        canctrl_byte
    }

    /// Read Status Instruction.
    ///
    /// ## NOTE
    ///
    /// Will show:
    ///
    /// RX0IF (CANINTF[0]) - bit 0
    /// RX1IF (CANINTF[1]) - bit 1
    /// TXREQ (TXB0CNTRL[3]) - bit 2
    /// TX0IF (CANINTF[2]) - bit 3
    /// TXREQ (TXB1CNTRL[3]) - bit 4
    /// TX1IF (CANINTF[3]) - bit 5
    /// TXREQ (TXB2CNTRL[3]) - bit 6
    /// TX2IF (CANINTF[4]) - bit 7
    fn read_status(&mut self) {
        let instruction_msg: [u8; 3] = [InstructionCommand::Status as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);
        self.cs.set_high();
    }

    /// RX Status Instruction.
    ///
    /// ## NOTE
    ///
    /// "Quick polling command that indicates filter match and message
    ///  type (standard, extended and/or remote) of received message"
    fn rx_status(&mut self) {
        let instruction_msg: [u8; 3] = [InstructionCommand::RxStatus as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);
        self.cs.set_high();
    }

    /// RTS - Request-To-Send instruction.
    /// This is called after a Load TX instruction has been run.
    /// It will work, when TXREQ bits are set to 1, indicating TX is pending.  
    fn request_to_send(&mut self, buffer: TXBN) {
        let rts_instruction: u8 = match buffer {
            TXBN::TXB0 => (InstructionCommand::RTS as u8 | 0x01),
            TXBN::TXB1 => (InstructionCommand::RTS as u8 | 0x02),
            TXBN::TXB2 => (InstructionCommand::RTS as u8 | 0x04),
        };

        let mut instruction_msg: [u8; 1] = [rts_instruction];
        self.cs.set_low();
        self.write(&instruction_msg);
        self.cs.set_high();
        //self.tx_pending(false);
        self.busy_tx[buffer.idx()] = true;
    }

    /// Performs 'Load TX' Instruction
    fn load_tx_buffer(&mut self, buffer: TXBN, mut data_in: CanMessage) {
        let instruction = match buffer {
            TXBN::TXB0 => InstructionCommand::LoadTX as u8,
            TXBN::TXB1 => InstructionCommand::LoadTX as u8 | 0x2,
            TXBN::TXB2 => InstructionCommand::LoadTX as u8 | 0x4,
        };

        let mut reg_offset = 0x01_u8;
        let mut address = buffer as u8;
        let mut data_bytes = data_in.to_bytes();

        /* Transaction START ------------------------------- */
        self.cs.set_low();
        self.spi.write(&[instruction]);

        for data in data_bytes {
            let next_address = address + reg_offset;
            self.spi.write(&[data]);
            reg_offset += 0x01;
        }
        self.cs.set_high();
        /* Transaction END ---------------------------------- */
    }

    /// Performs the 'Read Rx' Instruction.
    /// During the spi transfer, we send dont cares, since the address is auto
    /// incremented.
    fn read_rx_buffer(&mut self, buffer: RXBN) -> Result<[u8; 13], SPI::Error> {
        const DONT_CARE: u8 = 0x00;
        let mut rx_data: [u8; 3] = [0; 3];
        let mut rx_buffer = [0u8; 13];

        let mut instruction = match buffer {
            RXBN::RXB0 => InstructionCommand::ReadRxb0 as u8,
            RXBN::RXB1 => InstructionCommand::ReadRxb1 as u8,
        };

        /* Start of the transaction. */
        self.cs.set_low();
        let mut instruction_buf = [instruction];
        self.spi.write(&[instruction])?;
        self.spi.transfer(&mut rx_buffer, &[0; 13]).ok(); // Read the 14 bytes of CAN data
        self.cs.set_high();
        /* End of transaction. */

        Ok(rx_buffer)
    }

    /// Reset the MCP2515 and its registers to its default values.
    /// This is usually called, during startup, before configuration.
    fn reset_instruction(&mut self) {
        //defmt::println!("Calling Reset instruction over SPI");
        self.cs.set_low();
        self.spi.write(&[InstructionCommand::Reset as u8]);
        self.cs.set_high();
    }

    /// This would CLEAR the TXREQ bits indicating the tx buffer is not pending
    /// before writing. When `is_pending` flag is false, it indicate that
    /// transmit buffer is not pending transmission. Hence should be called
    /// before writing to the transmit buffer. Setting the flag to true,
    /// would flag a message buffer as being ready for transmission.
    ///
    /// ## TODO
    ///
    /// Add which TX buffer to set as ready or pending, by adding argument.
    /// This is important so we dont CLEAR or set something that is not ready.
    fn tx_pending(&mut self, is_pending: bool) -> &mut Self {
        const CLEAR: u8 = 0u8;
        const FILTERMASKOFF: u8 = 0b0110_0000;
        let mut reg_bits = 0u8;
        reg_bits |= 1 << 3;
        let bit_mask: u8 = 0b0000_1000;

        if (is_pending) {
            self.write_register(MCP2515Register::TXB0CTRL, bit_mask);
            //self.write_register(MCP2515Register::TXB1CTRL, reg_bits);
            //self.write_register(MCP2515Register::TXB2CTRL, reg_bits);
        } else {
            self.write_register(MCP2515Register::TXB0CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB1CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB2CTRL, CLEAR);
        }
        self
    }

    /// Method for checking if a transmit buffer `TXBN` is busy or not.
    fn tx_busy(&mut self, tx: TXBN) -> bool {
        let idx = tx.idx();
        let ret = unsafe { self.busy_tx.get_unchecked(idx) };
        *ret
    }

    /// Method for getting the available `TXBN` buffer. Starting from the higher
    /// priority TX buffer (`TXB0`).
    fn get_available_tx(&mut self) -> TXBN {
        if (!self.tx_busy(TXBN::TXB0)) {
            TXBN::TXB0
        } else if (!self.tx_busy(TXBN::TXB1)) {
            TXBN::TXB1
        } else {
            TXBN::TXB2
        }
    }

    /// This will set the RXBNCTRL.RXM bits, for setting the RX Operating Mode.
    ///
    /// ## NOTE
    ///
    /// RXM: Receive Buffer Operating Mode bits
    ///         ################################################
    ///         ## 11 = Turn mask/filters off; receive any message.
    ///         ## 10 = Receive only valid messages with extended identifiers
    /// that meet filter criteria.         ## 01 = Receive only valid
    /// messages with standard identifiers that meet filter criteria.
    ///         ## 00 = Receive all valid messages using either standard or
    /// extended identifiers that meet filter criteria.         ############
    /// ####################################
    fn set_rxm_mode(&mut self, rxm: ReceiveBufferMode) -> &mut Self {
        match (rxm) {
            ReceiveBufferMode::ReceiveAny => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0000_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0000_0000);
            }
            ReceiveBufferMode::FilterOffReceiveAny => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0110_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0110_0000);
            }
            ReceiveBufferMode::OnlyExtendedId => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0100_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0100_0000);
            }
            ReceiveBufferMode::OnlyStandardId => {
                // No rollover
                self.write_register(MCP2515Register::RXB0CTRL, 0b0000_0000);
                
                // With rollover if RXB0 is full.
                //self.write_register(MCP2515Register::RXB0CTRL, 0b0000_0100);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0000_0000);
            }
        }
        self
    }

    /// This would apply & configure the Can controller settings for the module.
    fn setup_configuration(&mut self, can_settings: &Mcp2515Settings) -> &mut Self {
        //defmt::info!("Setting up Configuration: ");

        self.reset_instruction();

        let canstat_reg = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let canctrl_byte = self.get_canctrl_mask(can_settings.canctrl);

        /* Write configuration bits to the register that needs to be setup. */
        self.write_register(MCP2515Register::CANCTRL, canctrl_byte);
        self.write_register(MCP2515Register::TXRTSCTRL, 0b0000_0011);
        self.write_register(MCP2515Register::BFPCTRL, 0b0000_0011);
        //self.write_register(MCP2515Register::RXB0CTRL, 0b0000_0011);

        /* Sanity check, reading so the configuration has written correctly */
        let canctrl_bits = self.read_register(MCP2515Register::CANCTRL, 0x00).unwrap();
        let txrtscttrl_reg = self
            .read_register(MCP2515Register::TXRTSCTRL, 0x00)
            .unwrap();
        let bfpctrl_reg = self.read_register(MCP2515Register::BFPCTRL, 0x00).unwrap();

        /* The requested mode must be verified by reading the OPMOD[2:0] bits
         * (CANSTAT[7:5]) */
        let canstat_new = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        //defmt::println!("CANSTAT after reset instruction is: {:08b}", canstat_reg);

        self
    }

    /// This change the mode and set it to Normal mode for sending and receiving
    /// on the CAN bus.
    pub fn activate_canbus(&mut self) {
        const ONE_SHOT_MODE: bool = true;
        if (self.can_settings.canctrl.mode != OperationTypes::Normal) {
            let canctrl_settings = SettingsCanCtrl::new(
                OperationTypes::Normal,
                true,
                CLKPRE::DIV1,
                false,
                ONE_SHOT_MODE,
            );
            self.can_settings.canctrl = canctrl_settings;
            self.change_settings(self.can_settings);
        }
    }

    /// Transmit frame on CAN bus, by loading TXBN buffer, and write
    /// 'Request-To-Send' instruction.
    ///
    /// ## TODO
    ///
    /// Add logic for targeting TX buffers to read, that is not busy...
    fn transmit_can(&mut self, can_msg: &CanMessage) {
        let mut can_frame = CanMessage::new(can_msg.id, &can_msg.data).unwrap();
        let txbn = self.get_available_tx();

        self.load_tx_buffer(txbn, can_frame); // How should I handle buffer target for transmission
        self.request_to_send(txbn);
    }

    /// Receive CAN frame by reading the appropriate RXBN buffer.
    ///
    /// ## TODO
    ///
    /// Write logic for picking appropriate RXBN buffer to read.
    /// Based on CANINTF, RXNCTRL.FILT bits, or through RX status instruction.
    fn receive_can(&mut self) -> CanMessage {
        let rx = unsafe { self.active_rx.0.unwrap_unchecked() };
        let rx_buffer = self.read_rx_buffer(rx).unwrap();
        let mut frame = CanMessage::try_from(rx_buffer).unwrap();
        //defmt::info!("Can bus received the Frame:");
        //frame.print_frame();
        frame
    }

    /// Setting up the MCP speed and bitrate of the MCP2515 module.
    fn setup_bitrate(&mut self) -> &mut Self {
        //TODO: - Fix parsing of can_settings to set other MCP speed and can bitrates

        /* Reference: https://github.com/autowp/arduino-mcp2515/blob/master/mcp2515.h
         * #define MCP_16MHz_125kBPS_CFG1 (0x03)
         * #define MCP_16MHz_125kBPS_CFG2 (0xF0)
         * #define MCP_16MHz_125kBPS_CFG3 (0x86)
         */

        //#define MCP_8MHz_1000kBPS_CFG1 (0x00)
        //#define MCP_8MHz_1000kBPS_CFG2 (0x80)
        //#define MCP_8MHz_1000kBPS_CFG3 (0x80)
        self.write_register(MCP2515Register::CNF1, 0x00);
        self.write_register(MCP2515Register::CNF2, 0x80);
        self.write_register(MCP2515Register::CNF3, 0x80);

        // This would set the MCP speed of 8MHz and a bitrate of 125kBPS
        //self.write_register(MCP2515Register::CNF1, 0x01);
        //self.write_register(MCP2515Register::CNF2, 0xB1);
        //self.write_register(MCP2515Register::CNF3, 0x85);

        self
    }

    /// Sets the acceptance filters in the peripheral.
    ///
    /// ## Note
    ///
    /// For this version of the crate we only support one filter per buffer.
    /// Future releases might support more but as this is developed as a part of
    /// a project this will be left was future work.
    ///
    /// ## Panics
    ///
    /// This function panics if the mask of filter exceed 11 bits.
    /// It also panics if the mask will ignore parts of the filter.
    ///
    /// None of these panics will happen in release.
    pub fn filter_message_id(&mut self, channel: RXBN, mask: u16, filter: u16) -> &mut Self {
        // Debug assert here. This will be run during init.
        // If
        debug_assert!(
            mask & filter == filter,
            "Mask will be at least partially canceled by the filter"
        );
        debug_assert!((filter >> 5) < 1 << 11, "Filter must be an 11 bit integer");
        debug_assert!((mask >> 5) < 1 << 11, "Mask must be an 11 bit integer");
        let ((mask_reg_high, mask_reg_low), (filter_reg_high, filter_reg_low)) = match channel {
            // RXB0 have the associated; RXM0, RXF0, RXF1.
            RXBN::RXB0 => (
                (MCP2515Register::RXM0SIDH, MCP2515Register::RXM0SIDL),
                (MCP2515Register::RXF0SIDH, MCP2515Register::RXF0SIDL),
            ),
            // RXB1 have the associated; RXM1, RXF2, RXF3, RXF4, RXF5.
            RXBN::RXB1 => (
                (MCP2515Register::RXM1SIDH, MCP2515Register::RXM1SIDL),
                (MCP2515Register::RXF2SIDH, MCP2515Register::RXF2SIDL),
            ),
        };

        let rxmn_sidh = ((mask >> 8) & 0b1111_1111) as u8;
        let rxmn_sidl = (mask & 0b1110_0000) as u8;
        let rxfn_sidh = ((filter >> 8) & 0b1111_1111) as u8;
        let rxfn_sidl = (filter & 0b1110_0000) as u8;

        self.write_register(mask_reg_high, rxmn_sidh);
        self.write_register(mask_reg_low, rxmn_sidl);
        self.write_register(filter_reg_high, rxfn_sidh);
        self.write_register(filter_reg_low, rxfn_sidl);

        self
    }

    /// Parsing the provided `MCP2515Settings` struct, and enable interrupts.
    ///
    /// ## Note
    ///
    /// "When a message is moved into either of the receive
    ///    buffers, the appropriate CANINTF.RXnIF bit is set. This
    ///    bit must be cleared by the MCU in order to allow a new
    ///    message to be received into the buffer. This bit
    ///    provides a positive lockout to ensure that the MCU has
    ///    finished with the message before the MCP2515
    ///    attempts to load a new message into the receive buffer.
    ///    If the CANINTE.RXnIE bit is set, an interrupt will be
    ///    generated on the INT pin to indicate that a valid
    ///    message has been received."
    pub fn setup_interrupt(&mut self, can_settings: &Mcp2515Settings) -> &mut Self {
        //let settings = can_settings.enable_interrupts(&[
        //    CanInte::MERRE,
        //    CanInte::ERRIE,
        //    CanInte::TX2IE,
        //    CanInte::TX1IE,
        //    CanInte::TX0IE,
        //    CanInte::RX1IE,
        //    CanInte::RX0IE,
        //]);
        //
        //can_settings.enable_interrupts(interrupt_types)
        self.can_settings.interrupts = can_settings.interrupts;
        self.write_register(MCP2515Register::CANINTE, can_settings.interrupts);
        self.read_register(MCP2515Register::CANINTE, 0x00);

        self
    }

    /// This method, would clear the CANINTF at `bit_pos` param.
    /// It send the Bit Modify instruction with mask and data bytes.
    fn clear_interrupt_flag(&mut self, bit_pos: u8) {
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        let mut mask_bytes: u8 = self.can_settings.interrupts; // enabled target interrupt bits.
        let mut data_bytes = canintf ^ (1 << bit_pos); //0bxxxx_1011 for clearing TX0IF
                                                       //
        //defmt::warn!("GPIOTE: Coool new things {:#05b} & {:#05b}",mask_bytes,data_bytes);

        self.write_register(MCP2515Register::CANINTF, 0x0);
    //
    //    self.cs.set_low();
    //    self.spi.write(&[
    //        InstructionCommand::Modify as u8,
    //        MCP2515Register::CANINTF as u8,
    //        //0b11111,
    //        mask_bytes,
    //        data_bytes,
    //    ]);
    //    self.cs.set_high();
    }

    /// Decodes the CANSTAT.ICOD bits, and map to matched `InterruptFlagCode`.
    ///
    /// # Panics
    ///
    /// Would panic if `read_register` method fails.
    ///
    /// # Errors
    ///
    /// Will return appropriate error type, if it fails to decode the
    /// `InterruptFlagCode`.
    pub fn interrupt_decode(&mut self) -> Result<InterruptFlagCode, Mcp2515Error> {
        let canstat = self
            .read_register(MCP2515Register::CANSTAT, 0x00)
            .map_err(|_| Mcp2515Error::SPIError)?;
        let mut interrupt_code = (canstat & 0b0000_1110) >> 1; // clear OPMOD bits and shift right by 1.

        //TODO: - Clean up code, by removing overhead to spi write and read.
        //Also the SPI instruction "RX STATUS", can check:
        //  - "No RX message"
        //  - "Message in RXB0"
        //  - "Message in RXB1"
        //  - "Messages in both buffers*" --> Could be useful for the handle_interrupt logic.

        match InterruptFlagCode::try_from(interrupt_code) {
            Ok(flag_code) => {
                //defmt::info!("Received thef Interrupt Code: {:?}", flag_code);
                Ok(flag_code)
            }
            Err(e) => {
                //defmt::error!("Failed to decode InterruptFlagCode: {:?}", e);
                Err(Mcp2515Error::MessageErrorInterrupt)
            }
        }
    }

    //WARN: - Just for debbuing, remove when done. 
    pub fn read_canintf(&mut self) -> u8 {
        self.read_register(MCP2515Register::CANINTF, 0x00).unwrap()
    }

    fn interrupt_code(&mut self) -> Result<InterruptFlagCode, Mcp2515Error> {
        let canstat = self
            .read_register(MCP2515Register::CANSTAT, 0x00)
            .map_err(|_| Mcp2515Error::SPIError)?;
        let mut interrupt_code = (canstat & 0b0000_1110) >> 1; // clear OPMOD bits and shift right by 1.

        todo!()
    }

    /// This just prints out, register bits related to a received error.
    ///
    /// ## NOTE
    ///
    /// EFLG register - bits related to a specific ERROR FLAG.
    /// TEC - Transmit Error Count.
    /// REC - Receive Error Count.
    fn error_handling(&mut self) {
        //let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let errorflag_reg = self.read_register(MCP2515Register::EFLG, 0x00).unwrap();
        let txbnctrl = self.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        let tec = self.read_register(MCP2515Register::TEC, 0x00).unwrap();
        let rec = self.read_register(MCP2515Register::REC, 0x00).unwrap();

        let mut eflags_output = [EFLG::UNKNOWN; 8];
        let eflgs_count = Self::eflg_decode(errorflag_reg, &mut eflags_output);
        let eflags_to_handle = &eflags_output[0..eflgs_count];

        let mut tx_req: bool = false;
        let mut tx_err: bool = false;
        let mut message_lost: bool = false;

        if (txbnctrl & (1 << 4) != 0) {
            tx_err = true;
        }
        if (txbnctrl & (1 << 3) != 0) {
            tx_req = true;
        }
        if (txbnctrl & (1 << 5) != 0) {
            message_lost = true;
        }

        //defmt::info!(
        //    "Error Decode Info: \nTXREQ = {:?}, TXERR = {:?}, MLOA = {:?}",
        //    tx_req,
        //    tx_err,
        //    message_lost
        //);
        defmt::info!("TEC status: {:08b}", tec);
        defmt::info!("REC status: {:08b}", rec);
        //defmt::info!("CANINTF register bits: {:08b}", canintf);
        defmt::info!("CANSTAT register bits: {:08b}", canstat);
        defmt::info!("EFLG register bits: {:08b}", errorflag_reg);
        //defmt::info!("EFLG: {:?} to handle!", eflags_to_handle);

    }

    /// Decodes the EFLG register...
    ///
    /// ## NOTE
    ///
    /// Still under progress!
    #[allow(clippy::cast_possible_truncation)]
    #[allow(clippy::needless_range_loop)]
    #[allow(clippy::cast_sign_loss)]
    fn eflg_decode(eflg_bits: u8, eflags: &mut [EFLG]) -> usize {
        let mut count: usize = 0;

        for bit_pos in 0..8 {
            if (eflg_bits & (1 << bit_pos) != 0) {
                //defmt::info!("Found EFLG: {:?}", EFLG::from_u8(bit_pos as u8));
                eflags[bit_pos] = EFLG::from_u8(bit_pos as u8);
                count += 1;
            }
        }
        //defmt::info!("eflg_decode count: {:?}", count);
        count
    }

    fn message_error_check(&mut self, canintf: u8) {
        if (canintf & (1 << 7) != 0) {
            //defmt::info!("Message Error Interrupt occurred (MERRE)!");
            // Handle message error below...

            self.clear_interrupt_flag(7);
        }
    }

    /// This would parse the received interrupt, triggered by the GPIOTE
    /// channel. Pass the decoded interrupt from `interrupt_decode()` and
    /// handle it. E.g., clear necessary regs such as clearing the
    /// appropriate CANINTF bits.
    ///
    ///
    /// ## TODO
    ///
    /// - Fix how different TX and RX buffers should be handled.
    /// - Write handle logic for: `ErrorInterrupt`, `WakeUpInterrupt`, TXB1,
    ///   TXB2.
    ///
    /// # Panics
    ///
    /// Could panic if `read_register` method fails to read.
    pub fn handle_interrupt(
        &mut self,
        received_interrupt: InterruptFlagCode,
    ) -> Option<CanMessage> {
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        self.message_error_check(canintf);
        //defmt::println!("CANINTF register bits: {:08b}", canintf);
        //defmt::warn!("{:08b}", received_interrupt as u8); 
        match received_interrupt {
            InterruptFlagCode::NoInterrupt => None,
            InterruptFlagCode::ErrorInterrupt => {
                //defmt::warn!("Received ERRIF, with flags...");
                self.error_handling();
                self.clear_interrupt_flag(5);
                None
            }
            InterruptFlagCode::WakeUpInterrupt => {
                //defmt::println!("Wakeup interrupt occurred!");
                None
            }
            InterruptFlagCode::TXB0Interrupt => {
                //defmt::println!("TXB0 successfully transmitted a message!");
                self.busy_tx[TXBN::TXB0.idx()] = false;

                self.clear_interrupt_flag(2);
                let all_handled = self.interrupt_is_cleared();
                None
            }
            InterruptFlagCode::TXB1Interrupt => {
                //defmt::println!("TXB1 successfully transmitted a message!");
                self.busy_tx[TXBN::TXB1.idx()] = false;
                self.clear_interrupt_flag(3);
                None
            }
            InterruptFlagCode::TXB2Interrupt => {
                //defmt::println!("TXB2 successfully transmitted a message!");
                self.busy_tx[TXBN::TXB2.idx()] = false;
                self.clear_interrupt_flag(4);
                None
            }
            InterruptFlagCode::RXB0Interrupt => {
                //defmt::println!("RXB0 received a message!");
                self.active_rx = (Some(RXBN::RXB0), true);
                let mut frame = self.receive().unwrap();
                //let mut f = self.receive().ok()

                //let high_prio_id = StandardId::new(0x0).unwrap();
                //let id_high_prio = Id::Standard(high_prio_id);
                //let mut frame_response = CanMessage::new(id_high_prio, frame.data()).unwrap();

                //self.transmit(&frame_response);
                self.clear_interrupt_flag(0);
                Some(frame)
            }
            InterruptFlagCode::RXB1Interrupt => {
                //defmt::println!("RXB1 received a message!");
                self.active_rx = (Some(RXBN::RXB1), true);
                let mut frame = self.receive().unwrap();

                //self.transmit(&frame_response);
                self.clear_interrupt_flag(1);
                Some(frame)
            }
        }
    }

    /// This would check if all interrupt (interrupt flags) has been handled.
    /// Will return true, if all has been handled, else false.
    ///
    /// # Panics
    ///
    /// Will panic if reading the `CANINTF` register fails.
    pub fn interrupt_is_cleared(&mut self) -> bool {
        const ZERO: u8 = 0u8;
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        canintf == ZERO
    }

    /// Returns a simple event
    pub fn interrupt_manager<'a>(&'a mut self) -> CanEventManager<'a, SPI, PIN, PININT> {
        CanEventManager::new(self)
    }
}

/// A simple event manager that iterates over all of the events in the `Mcp2515` device.
///
/// This is created with [`interrupt_managert`](Mcp2515Driver::interrupt_manager).
/// And allows you to manage the events in a simple manner.
///
/// ```ignore
/// let mut manager = driver.interrupt_manager();
/// while let Some(event) = manager.next() {
///     let _ = event.handle();
/// }
/// ```
///
/// Or if you want to receive frames you can simply do
///
/// ```ignore
/// let mut manager = driver.interrupt_manager();
/// let mut received_message = None;
/// while let Some(event) = manager.next() {
///     if let Some(message) = event.handle() {
///         received_message = Some(message)
///     }
/// }
/// ```
pub struct CanEventManager<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> {
    can: &'bus mut Mcp2515Driver<SPI, PIN, PININT>,
}

impl<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin>
    CanEventManager<'bus, SPI, PIN, PININT>
{
    /// Constructs a new event manager.
    fn new(can: &'bus mut Mcp2515Driver<SPI, PIN, PININT>) -> Self {
        Self { can }
    }
}

impl<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin>
    CanEventManager<'bus, SPI, PIN, PININT>
{
    /// Check the next interrupt flag code to process. 
    pub fn next<'a>(&'a mut self) -> Option<CanEvent<'a, SPI, PIN, PININT>> {
        if self.can.interrupt_is_cleared() {
            return None;
        }

        let icode: InterruptFlagCode = self.can.interrupt_decode().ok()?;
        //defmt::info!("GPIOTE icode : {:#05b}",icode as u8);

        Some(CanEvent::new(&mut self.can, icode))
    }
}

/// Represents a single can event that needs to be handled.
///
/// This is generated by [`CanEventManager`] and is handled using [`handle`](CanEvent::handle).
/// This can optionally return a frame if one was received.
pub struct CanEvent<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> {
    can: &'bus mut Mcp2515Driver<SPI, PIN, PININT>,
    pub event_code: InterruptFlagCode,
}

impl<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin>
    CanEvent<'bus, SPI, PIN, PININT>
{
    /// Constructs a new event manager.
    fn new(can: &'bus mut Mcp2515Driver<SPI, PIN, PININT>, event_code: InterruptFlagCode) -> Self {
        Self { can, event_code }
    }

    /// Handles the incoming event.
    ///
    /// If this was a receive event it will return the contained frame.
    pub fn handle(mut self) -> Option<CanMessage> {
        //defmt::info!("GPIOTE: Manager handle, event_code : {:#05b}", self.event_code as u8);
        self.can.handle_interrupt(self.event_code)
    }
}

impl<'bus, SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> core::fmt::Debug
    for CanEvent<'bus, SPI, PIN, PININT>
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.event_code)
    }
}
