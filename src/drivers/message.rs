//! CAN frame message support.
//! ## Usage

#![no_main]
#![no_std]
#![allow(unused)]

use core::str;

//use cortex_m::asm as _;
//use cortex_m_rt::entry;
//use defmt::{write, Format, Formatter};
//use defmt_rtt as _;
use digital::ErrorKind;
use embedded_can::{nb::Can, Error, Frame, StandardId};
use embedded_hal::{digital, digital::OutputPin, spi::SpiBus};
use nb;

//use nrf52840_hal::{
//    self as _,
//    comp::OperationMode,
//    gpio::{Level, Port},
//    pac::i2s::config::format,
//    spi,
//};

/// Represent a CAN message.
#[derive(Debug)]
#[allow(clippy::module_name_repetitions)]
pub struct CanMessage {
    /// CAN identifier, can be either Standard or Extended.
    pub id: embedded_can::Id,
    /// Data Length Code (DLC) - length of the data byte fields.
    pub dlc: u8,
    /// Data bytes associated to a CAN frame.
    ///
    /// Every RX/TX buffer, can use the data register D7 to D0 (8 bytes max).
    pub data: [u8; 8], // D7 - D0
}

impl Frame for CanMessage {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        Self::new(id.into(), data)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        None
    }

    fn is_extended(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> embedded_can::Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn data(&self) -> &[u8] {
        &self.data[0..self.dlc()]
    }
}

/// Here goes custom logic for the `CanMessage` struct type
impl CanMessage {
    /// Creates a new CAN message.
    ///
    /// Would fail, if the provided data, exceedes 8 bytes.
    #[allow(clippy::cast_possible_truncation)]
    pub fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let data_length_code = data.len() as u8;
        let mut can_msg = Self {
            id: id.into(),
            dlc: data_length_code,
            data: [0u8; 8],
        };
        can_msg.data[..data_length_code as usize].copy_from_slice(data);
        Some(can_msg)
    }

    /// Converts the SIDH + SIDL of 11 bits to 16 bits.
    #[allow(clippy::cast_possible_truncation)]
    pub fn id_raw(&mut self) -> u16 {
        let mut raw_id: u16;
        if let embedded_can::Id::Standard(id) = self.id {
            raw_id = id.as_raw();
        } else {
            raw_id = 0;
        }

        //TODO: - Replace with:
        let full_standardid_as_u16 = raw_id << 5;

        let sid_high = (raw_id >> 3) as u8; // Most significant byte
        let sid_low = (raw_id as u8 & 0x07) << 5; // Least significant byte, 0x07 = 00000_0111
        let combined = ((u16::from(sid_high)) << 8) | u16::from(sid_low);
        full_standardid_as_u16
    }

    /// Prints the ID, DLC, Data fields of a CAN message frame.
    pub fn print_frame(&mut self) {
        let id = self.id_raw() >> 5;
        let dlc = self.dlc;
        let data = self.data();
        //defmt::info!("Id({:x}), DLC({:x}), Data({:08b})", id, dlc, data);
    }

    /// Maps a CAN message to a byte sequence.
    /// 0 => SIDH
    /// 1 => SIDL
    /// 2 => EID8
    /// 3 => EID0
    /// 4 => DLC
    /// 5 => D0
    /// 6 => D1
    /// 7 => D2
    /// 8 => D3
    /// 9 => D4
    /// 10 => D5
    /// 11 => D6
    /// 12 => D7
    #[allow(clippy::cast_possible_truncation)]
    pub fn to_bytes(&mut self) -> [u8; 13] {
        let mut byte_frame: [u8; 13] = [0; 13];
        let mut raw_id: u16;

        if let embedded_can::Id::Standard(id) = self.id {
            raw_id = id.as_raw();
        } else {
            raw_id = 0;
        }

        // Split the u16 raw_id into two u8 values
        let sid_high = (raw_id >> 3) as u8; // Most significant byte
        let sid_low = (raw_id as u8 & 0x07) << 5; // Least significant byte, 0x07 = 00000_0111

        //TODO: - Fix the parsing logic for the extended id bits...
        let extended_id8: u8 = 0;
        let extended_id0: u8 = 0;

        let data_start = 5_usize;
        let data_end = data_start + self.dlc();

        byte_frame[0..data_start].copy_from_slice(&[
            sid_high,
            sid_low,
            extended_id8,
            extended_id0,
            self.dlc() as u8,
        ]);

        byte_frame[data_start..data_end].copy_from_slice(&self.data[0..self.dlc()]);
        byte_frame
    }
}

///This From trait would parse a type T into a `CanMessage` frame.
impl TryFrom<[u8; 13]> for CanMessage {
    type Error = &'static str;

    #[allow(clippy::cast_possible_truncation)]
    fn try_from(value: [u8; 13]) -> Result<Self, Self::Error> {
        let mut id_bytes: [u8; 2];
        let mut raw_id: u16 = 0u16;

        let slice_sid_high = u16::from(value[0]);
        let slice_sid_low = u16::from(value[1] & 0xE0);
        let dlc = value[4];

        let data_start = 5_usize;
        let data_end = data_start + dlc as usize;

        let mut data = &value[data_start..];

        // Shift MSB u16 SIDH part and OR the LSB u16 SIDL part
        raw_id = ((slice_sid_high << 3) + (slice_sid_low >> 5));

        let frame_id = StandardId::new(raw_id).ok_or("Invalid ID: out of 11-bit range")?;
        Self::new(embedded_can::Id::Standard(frame_id), data)
            .ok_or("Failed to create CanMessage frame")
    }
}
