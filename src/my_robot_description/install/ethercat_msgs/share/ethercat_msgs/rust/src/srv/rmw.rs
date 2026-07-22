#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__SetSdo_Request() -> *const std::ffi::c_void;
}

#[link(name = "ethercat_msgs__rosidl_generator_c")]
extern "C" {
    fn ethercat_msgs__srv__SetSdo_Request__init(msg: *mut SetSdo_Request) -> bool;
    fn ethercat_msgs__srv__SetSdo_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Request>, size: usize) -> bool;
    fn ethercat_msgs__srv__SetSdo_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Request>);
    fn ethercat_msgs__srv__SetSdo_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetSdo_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Request>) -> bool;
}

// Corresponds to ethercat_msgs__srv__SetSdo_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub sdo_data_type: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_value: rosidl_runtime_rs::String,

}



impl Default for SetSdo_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ethercat_msgs__srv__SetSdo_Request__init(&mut msg as *mut _) {
        panic!("Call to ethercat_msgs__srv__SetSdo_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetSdo_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetSdo_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetSdo_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ethercat_msgs/srv/SetSdo_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__SetSdo_Request() }
  }
}


#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__SetSdo_Response() -> *const std::ffi::c_void;
}

#[link(name = "ethercat_msgs__rosidl_generator_c")]
extern "C" {
    fn ethercat_msgs__srv__SetSdo_Response__init(msg: *mut SetSdo_Response) -> bool;
    fn ethercat_msgs__srv__SetSdo_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Response>, size: usize) -> bool;
    fn ethercat_msgs__srv__SetSdo_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Response>);
    fn ethercat_msgs__srv__SetSdo_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetSdo_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetSdo_Response>) -> bool;
}

// Corresponds to ethercat_msgs__srv__SetSdo_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetSdo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_message: rosidl_runtime_rs::String,

}



impl Default for SetSdo_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ethercat_msgs__srv__SetSdo_Response__init(&mut msg as *mut _) {
        panic!("Call to ethercat_msgs__srv__SetSdo_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetSdo_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__SetSdo_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetSdo_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetSdo_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ethercat_msgs/srv/SetSdo_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__SetSdo_Response() }
  }
}


#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__GetSdo_Request() -> *const std::ffi::c_void;
}

#[link(name = "ethercat_msgs__rosidl_generator_c")]
extern "C" {
    fn ethercat_msgs__srv__GetSdo_Request__init(msg: *mut GetSdo_Request) -> bool;
    fn ethercat_msgs__srv__GetSdo_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Request>, size: usize) -> bool;
    fn ethercat_msgs__srv__GetSdo_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Request>);
    fn ethercat_msgs__srv__GetSdo_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSdo_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Request>) -> bool;
}

// Corresponds to ethercat_msgs__srv__GetSdo_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub sdo_data_type: rosidl_runtime_rs::String,

}



impl Default for GetSdo_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ethercat_msgs__srv__GetSdo_Request__init(&mut msg as *mut _) {
        panic!("Call to ethercat_msgs__srv__GetSdo_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSdo_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSdo_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSdo_Request where Self: Sized {
  const TYPE_NAME: &'static str = "ethercat_msgs/srv/GetSdo_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__GetSdo_Request() }
  }
}


#[link(name = "ethercat_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__GetSdo_Response() -> *const std::ffi::c_void;
}

#[link(name = "ethercat_msgs__rosidl_generator_c")]
extern "C" {
    fn ethercat_msgs__srv__GetSdo_Response__init(msg: *mut GetSdo_Response) -> bool;
    fn ethercat_msgs__srv__GetSdo_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Response>, size: usize) -> bool;
    fn ethercat_msgs__srv__GetSdo_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Response>);
    fn ethercat_msgs__srv__GetSdo_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetSdo_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetSdo_Response>) -> bool;
}

// Corresponds to ethercat_msgs__srv__GetSdo_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetSdo_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_message: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_value_string: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdo_return_value: f64,

}



impl Default for GetSdo_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ethercat_msgs__srv__GetSdo_Response__init(&mut msg as *mut _) {
        panic!("Call to ethercat_msgs__srv__GetSdo_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetSdo_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ethercat_msgs__srv__GetSdo_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetSdo_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetSdo_Response where Self: Sized {
  const TYPE_NAME: &'static str = "ethercat_msgs/srv/GetSdo_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ethercat_msgs__srv__GetSdo_Response() }
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


