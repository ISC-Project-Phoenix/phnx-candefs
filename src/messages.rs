use concat_arrays::concat_arrays;
use embedded_hal::can::Id;
use embedded_hal::can::{ExtendedId, Frame};

/// Errors caused by frame conversion
#[derive(Copy, Clone, Debug)]
pub enum ConvertErr {
    InvalidFrame,
}

pub trait IscFrame {
    /// Frame ID.
    const ID: u32;

    /// Converts self into a CAN frame.
    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr>
    where
        Self: Sized,
    {
        T::new(ExtendedId::new(Self::ID).unwrap(), &[]).ok_or(ConvertErr::InvalidFrame)
    }
}

/// All messages used in Phoenix.
#[derive(Copy, Clone, Debug)]
pub enum CanMessage {
    /// Tells the interface board to stop sending messages from ROS to the CAN network. The interface board should send a message to the PC, where ROS will state transition to teleop.
    /// There will be no auton enable message, rather you will need to toggle auton via a physical switch.
    AutonDisable(AutonDisable),
    /// Sets the brake to a certain percent engagement.
    SetBrake(SetBrake),
    ///  Prevents further braking messages from being sent from the interface to the bus.
    LockBrake(LockBrake),
    /// Lets more braking messages be sent to the bus, if locked.
    UnlockBrake(UnlockBrake),
    /// Sets the steering motor to a certain angle, and holds it.
    SetAngle(SetAngle),
    /// Contains the current steering angle of the motor.
    GetAngle(GetAngle),
    /// Sets the motor speed to the contained speed percent.
    SetSpeed(SetSpeed),
    /// Encoder ticks since last CAN message, as well as current velocity.
    EncoderCount(EncoderCount),
    /// Engages training mode. Any node that receives this should begin to relay data on the CAN bus for data collection,
    /// if applicable. There is no way to exit training mode, rather you power cycle CAN.
    TrainingMode(TrainingMode),
}

impl CanMessage {
    /// Converts a CAN frame into a defined frame. Errors if an undefined id is used.
    pub fn from_frame(value: impl Frame) -> Result<Self, ConvertErr> {
        if let Id::Extended(id) = value.id() {
            match id.as_raw() {
                AutonDisable::ID => Ok(CanMessage::AutonDisable(AutonDisable {})),
                SetBrake::ID => Ok(CanMessage::SetBrake(SetBrake {
                    percent: value.data()[0],
                })),
                LockBrake::ID => Ok(CanMessage::LockBrake(LockBrake {})),
                UnlockBrake::ID => Ok(CanMessage::UnlockBrake(UnlockBrake {})),
                SetAngle::ID => Ok(CanMessage::SetAngle(SetAngle {
                    angle: f32::from_le_bytes(
                        value.data()[0..4]
                            .try_into()
                            .map_err(|_| ConvertErr::InvalidFrame)?,
                    ),
                })),
                GetAngle::ID => Ok(CanMessage::GetAngle(GetAngle {
                    angle: f32::from_le_bytes(
                        value.data()[0..4]
                            .try_into()
                            .map_err(|_| ConvertErr::InvalidFrame)?,
                    ),
                })),
                SetSpeed::ID => Ok(CanMessage::SetSpeed(SetSpeed {
                    percent: value.data()[0],
                })),
                EncoderCount::ID => Ok(CanMessage::EncoderCount(EncoderCount {
                    count: u16::from_le_bytes(
                        value.data()[0..2]
                            .try_into()
                            .map_err(|_| ConvertErr::InvalidFrame)?,
                    ),
                    velocity: f32::from_le_bytes(
                        value.data()[2..6]
                            .try_into()
                            .map_err(|_| ConvertErr::InvalidFrame)?,
                    ),
                })),
                TrainingMode::ID => Ok(CanMessage::TrainingMode(TrainingMode {})),
                _ => Err(ConvertErr::InvalidFrame),
            }
        } else {
            Err(ConvertErr::InvalidFrame)
        }
    }
}

/// Tells the interface board to stop sending messages from ROS to the CAN network. The interface board should send a message to the PC, where ROS will state transition to teleop.
/// There will be no auton enable message, rather you will need to toggle auton via a physical switch.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct AutonDisable {}

impl IscFrame for AutonDisable {
    const ID: u32 = 0x0000000;
}

/// Sets the brake to a certain percent engagement.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct SetBrake {
    pub percent: u8,
}

impl IscFrame for SetBrake {
    const ID: u32 = 0x0000001;

    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr> {
        let data = [self.percent];
        T::new(ExtendedId::new(Self::ID).unwrap(), &data).ok_or(ConvertErr::InvalidFrame)
    }
}

/// Prevents further braking messages from being sent from the interface to the bus.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct LockBrake {}

impl IscFrame for LockBrake {
    const ID: u32 = 0x0000002;
}

/// Lets more braking messages be sent to the bus, if locked.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct UnlockBrake {}

impl IscFrame for UnlockBrake {
    const ID: u32 = 0x0000003;
}

/// Sets the steering motor to a certain angle, and holds it.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct SetAngle {
    /// Degrees, where left is negative, and right is positive.
    pub angle: f32,
}

impl IscFrame for SetAngle {
    const ID: u32 = 0x0000004;

    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr> {
        let data = self.angle.to_le_bytes();
        T::new(ExtendedId::new(Self::ID).unwrap(), &data).ok_or(ConvertErr::InvalidFrame)
    }
}

/// Contains the current steering angle of the motor.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct GetAngle {
    /// Degrees, where left is negative, and right is positive.
    pub angle: f32,
}

impl IscFrame for GetAngle {
    const ID: u32 = 0x0000005;

    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr> {
        let data = self.angle.to_le_bytes();
        T::new(ExtendedId::new(Self::ID).unwrap(), &data).ok_or(ConvertErr::InvalidFrame)
    }
}

impl GetAngle {
    /// Converts the steering angle to ackermann wheel angle.
    pub fn ackermann_angle(&self) -> f32 {
        self.angle * 2.62 + -0.832
    }
}

/// Sets the motor speed to the contained speed percent.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct SetSpeed {
    pub percent: u8,
}

impl IscFrame for SetSpeed {
    const ID: u32 = 0x0000006;

    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr> {
        let data = &[self.percent];
        T::new(ExtendedId::new(Self::ID).unwrap(), &data[..]).ok_or(ConvertErr::InvalidFrame)
    }
}

/// Encoder ticks since last CAN message, as well as current velocity.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct EncoderCount {
    pub count: u16,
    /// Speed in m/s.
    pub velocity: f32,
}

impl IscFrame for EncoderCount {
    const ID: u32 = 0x0000007;

    fn into_frame<T: Frame>(self) -> Result<T, ConvertErr> {
        let count = self.count.to_le_bytes();
        let vel = self.velocity.to_le_bytes();
        let data: [u8; core::mem::size_of::<u16>() + core::mem::size_of::<f32>()] =
            concat_arrays!(count, vel);

        T::new(ExtendedId::new(Self::ID).unwrap(), &data).ok_or(ConvertErr::InvalidFrame)
    }
}

/// Engages training mode. Any node that receives this should begin to relay data on the CAN bus for data collection,
/// if applicable. There is no way to exit training mode, rather you power cycle CAN.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct TrainingMode {}

impl IscFrame for TrainingMode {
    const ID: u32 = 0x0000008;
}

#[cfg(test)]
mod test {
    use super::*;
    extern crate std;
    use bxcan::Id::Extended;
    use std::prelude::rust_2021::*;

    #[test]
    fn test_steering_angle() {
        let frame: bxcan::Frame = GetAngle { angle: 4.818 }.into_frame().unwrap();

        // Test enum to frame
        if let Extended(id) = frame.id() {
            assert_eq!(id.as_raw(), 0x5);
        } else {
            assert!(false)
        }

        // Test frame to enum
        let conv = CanMessage::from_frame(frame).unwrap();

        if let CanMessage::GetAngle(g) = conv {
            assert_eq!(g.angle, 4.818);

            assert!((10.0..12.0).contains(&g.ackermann_angle()));
        } else {
            assert!(false)
        }
    }

    #[test]
    fn test_encoder() {
        let frame: bxcan::Frame = EncoderCount {
            count: 20,
            velocity: 10.2,
        }
        .into_frame()
        .unwrap();

        if let Extended(id) = frame.id() {
            assert_eq!(id.as_raw(), 0x7);
        } else {
            assert!(false)
        }

        let conv = CanMessage::from_frame(frame).unwrap();

        if let CanMessage::EncoderCount(ec) = conv {
            assert_eq!(ec.velocity, 10.2);
            assert_eq!(ec.count, 20);
        }
    }
}
