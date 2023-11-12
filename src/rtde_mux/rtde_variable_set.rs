use crate::raw_rtde::RTDEData;

/// A set of variables that can be used for reading (i.e. robot to host transmition).
/// 
/// In the future we may support a `derive` macro to automatically generate one for a struct.
pub trait RTDEReadVariableSet {
    /// Returns the names of the variables in the set.
    fn get_variables() -> Vec<String>;
    /// Creates a new structure based on value in RTDEData.
    /// 
    /// The values correspond to the variable in get_variables(), in the same order.
    fn fill_from_rtde_data(data: &[RTDEData]) -> Result<Self, String> where Self: Sized;
}

/// A set of variables that can be used for writing (i.e. host to robot transmition).
/// 
/// In the future we may support a `derive` macro to automatically generate one for a struct.
pub trait RTDEWriteVariableSet {
    /// Returns the names of the variables in the set.
    fn get_variables() -> Vec<String>;
    /// Converts the contents of `self` into RTDE variable values.
    /// 
    /// The values need to correspond the the variables in `get_variables()`, in the same order.
    fn convert_into_rtde_data(&self) -> Vec<RTDEData>;
}
