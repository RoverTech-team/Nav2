#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to ethercat_msgs__srv__SetSdo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetSdo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub master_id: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slave_position: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_index: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_subindex: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_data_type: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_value: std::string::String,

}



impl Default for SetSdo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetSdo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetSdo_Request {
  type RmwMsg = super::srv::rmw::SetSdo_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        master_id: msg.master_id,
        slave_position: msg.slave_position,
        sdo_index: msg.sdo_index,
        sdo_subindex: msg.sdo_subindex,
        sdo_data_type: msg.sdo_data_type.as_str().into(),
        sdo_value: msg.sdo_value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      master_id: msg.master_id,
      slave_position: msg.slave_position,
      sdo_index: msg.sdo_index,
      sdo_subindex: msg.sdo_subindex,
        sdo_data_type: msg.sdo_data_type.as_str().into(),
        sdo_value: msg.sdo_value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      master_id: msg.master_id,
      slave_position: msg.slave_position,
      sdo_index: msg.sdo_index,
      sdo_subindex: msg.sdo_subindex,
      sdo_data_type: msg.sdo_data_type.to_string(),
      sdo_value: msg.sdo_value.to_string(),
    }
  }
}


// Corresponds to ethercat_msgs__srv__SetSdo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetSdo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_message: std::string::String,

}



impl Default for SetSdo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetSdo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetSdo_Response {
  type RmwMsg = super::srv::rmw::SetSdo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        sdo_return_message: msg.sdo_return_message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        sdo_return_message: msg.sdo_return_message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      sdo_return_message: msg.sdo_return_message.to_string(),
    }
  }
}


// Corresponds to ethercat_msgs__srv__GetSdo_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSdo_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub master_id: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub slave_position: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_index: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_subindex: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_data_type: std::string::String,

}



impl Default for GetSdo_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSdo_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetSdo_Request {
  type RmwMsg = super::srv::rmw::GetSdo_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        master_id: msg.master_id,
        slave_position: msg.slave_position,
        sdo_index: msg.sdo_index,
        sdo_subindex: msg.sdo_subindex,
        sdo_data_type: msg.sdo_data_type.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      master_id: msg.master_id,
      slave_position: msg.slave_position,
      sdo_index: msg.sdo_index,
      sdo_subindex: msg.sdo_subindex,
        sdo_data_type: msg.sdo_data_type.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      master_id: msg.master_id,
      slave_position: msg.slave_position,
      sdo_index: msg.sdo_index,
      sdo_subindex: msg.sdo_subindex,
      sdo_data_type: msg.sdo_data_type.to_string(),
    }
  }
}


// Corresponds to ethercat_msgs__srv__GetSdo_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSdo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_value_string: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_value: f64,

}



impl Default for GetSdo_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetSdo_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetSdo_Response {
  type RmwMsg = super::srv::rmw::GetSdo_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        sdo_return_message: msg.sdo_return_message.as_str().into(),
        sdo_return_value_string: msg.sdo_return_value_string.as_str().into(),
        sdo_return_value: msg.sdo_return_value,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        sdo_return_message: msg.sdo_return_message.as_str().into(),
        sdo_return_value_string: msg.sdo_return_value_string.as_str().into(),
      sdo_return_value: msg.sdo_return_value,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      sdo_return_message: msg.sdo_return_message.to_string(),
      sdo_return_value_string: msg.sdo_return_value_string.to_string(),
      sdo_return_value: msg.sdo_return_value,
    }
  }
}






#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ethercat_msgs__srv__SetSdo() -> *const std::ffi::c_void;
}

// Corresponds to ethercat_msgs__srv__SetSdo
#[allow(missing_docs, non_camel_case_types)]
pub struct SetSdo;

impl rosidl_runtime_rs::Service for SetSdo {
    type Request = SetSdo_Request;
    type Response = SetSdo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ethercat_msgs__srv__SetSdo() }
    }
}




#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__ethercat_msgs__srv__GetSdo() -> *const std::ffi::c_void;
}

// Corresponds to ethercat_msgs__srv__GetSdo
#[allow(missing_docs, non_camel_case_types)]
pub struct GetSdo;

impl rosidl_runtime_rs::Service for GetSdo {
    type Request = GetSdo_Request;
    type Response = GetSdo_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__ethercat_msgs__srv__GetSdo() }
    }
}


