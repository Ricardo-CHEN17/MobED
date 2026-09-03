#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robot_interfaces__msg__MobEDCommand
/// 1. Driving Command
/// Target linear velocity (x, y) and angular velocity (z) for the robot body

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MobEDCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::Twist,

    /// 2. Posture Command
    /// Target posture variables for the balance controller
    /// Target body height from the ground (m)
    pub body_height: f64,

    /// Target roll angle (radians)
    pub body_roll: f64,

    /// Target pitch angle (radians)
    pub body_pitch: f64,

}



impl Default for MobEDCommand {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MobEDCommand::default())
  }
}

impl rosidl_runtime_rs::Message for MobEDCommand {
  type RmwMsg = super::msg::rmw::MobEDCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
        body_height: msg.body_height,
        body_roll: msg.body_roll,
        body_pitch: msg.body_pitch,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
      body_height: msg.body_height,
      body_roll: msg.body_roll,
      body_pitch: msg.body_pitch,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      twist: geometry_msgs::msg::Twist::from_rmw_message(msg.twist),
      body_height: msg.body_height,
      body_roll: msg.body_roll,
      body_pitch: msg.body_pitch,
    }
  }
}


