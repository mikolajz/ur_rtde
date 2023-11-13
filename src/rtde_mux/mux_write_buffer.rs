use std::collections::BTreeMap;
use std::mem::discriminant;
use crate::raw_rtde::RTDEData;

pub(crate) struct MuxWriteBuffer{
    current_values: BTreeMap<String, RTDEData>,
}

impl MuxWriteBuffer {
    pub(crate) fn new(
        init_values: &BTreeMap<String, RTDEData>,
    ) -> MuxWriteBuffer {
        MuxWriteBuffer{
            current_values: init_values.clone(),
        }
    }

    pub(crate) fn update(&mut self, names: &[String], values: &[RTDEData]) -> Result<(), String> {
        for (name, value) in names.iter().zip(values) {
            // TODO(mikolajz): can we achieve it without cloning the String and without modifying the state on invalid input?
            let previous_value = self.current_values.insert(name.clone(), value.clone()).ok_or_else(|| format!("Unsupported name {name}"))?;
            if discriminant(&previous_value) != discriminant(value) {
                return Err(format!("Value {name} changed type from {previous_value:?} to {value:?}"));
            }
        }
        Ok(())
    }

    pub(crate) fn values(&self) -> Vec<RTDEData> {
        self.current_values.values().cloned().collect()
    }
}

#[cfg(test)]
mod tests
{
    use super::*;
    use crate::raw_rtde::RTDEData;

    #[test]
    fn merges_values_correctly() {
        let mut init_values = BTreeMap::<String, RTDEData>::new();
        init_values.insert(String::from("a"), RTDEData::Int32(1));
        init_values.insert(String::from("b"), RTDEData::Int32(2));
        init_values.insert(String::from("c"), RTDEData::Uint64(3));

        let mut buffer = MuxWriteBuffer::new(&init_values);
        assert_eq!(buffer.values(), vec![RTDEData::Int32(1), RTDEData::Int32(2), RTDEData::Uint64(3)]);

        buffer.update(&[String::from("c"), String::from("a")], &[RTDEData::Uint64(4), RTDEData::Int32(5)]).expect("update failed");
        assert_eq!(buffer.values(), vec![RTDEData::Int32(5), RTDEData::Int32(2), RTDEData::Uint64(4)]);

        buffer.update(&[String::from("a"), String::from("b")], &[RTDEData::Int32(6), RTDEData::Int32(7)]).expect("update failed");
        assert_eq!(buffer.values(), vec![RTDEData::Int32(6), RTDEData::Int32(7), RTDEData::Uint64(4)]);
    }

    #[test]
    fn fails_on_type_change() {
        let mut init_values = BTreeMap::<String, RTDEData>::new();
        init_values.insert(String::from("a"), RTDEData::Int32(1));
        let mut buffer = MuxWriteBuffer::new(&init_values);

        buffer.update(&[String::from("a")], &[RTDEData::Uint64(4)]).expect_err("unexpectedly succeeded to change the type");
    }

    #[test]
    fn fails_on_invalid_name() {
        let mut init_values = BTreeMap::<String, RTDEData>::new();
        init_values.insert(String::from("a"), RTDEData::Int32(1));
        let mut buffer = MuxWriteBuffer::new(&init_values);

        buffer.update(&[String::from("x")], &[RTDEData::Uint64(4)]).expect_err("unexpectedly succeeded on wrong name");
    }

}
