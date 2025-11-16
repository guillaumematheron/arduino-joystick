use serde::Deserialize;

#[derive(Debug, Deserialize)]
pub struct FGFloatProperty {
    pub value: f64,
}

#[derive(Debug, Deserialize)]
pub struct FGBoolProperty {
    pub value: bool,
}

#[derive(Debug, Deserialize)]
pub struct FGIntProperty {
    pub value: i32,
}


pub fn get_property_float(path: &str) -> Option<f64> {
    let url = format!("http://localhost:8080/json{}", path);
    let res = reqwest::blocking::get(&url).ok()?;
    let val: FGFloatProperty = res.json().ok()?;
    Some(val.value)
}

pub fn get_property_bool(path: &str) -> Option<bool> {
    let url = format!("http://localhost:8080/json{}", path);
    let res = reqwest::blocking::get(&url).ok()?;
    let val: FGBoolProperty = res.json().ok()?;
    Some(val.value)
}

pub fn get_property_int(path: &str) -> Option<i32> {
    let url = format!("http://localhost:8080/json{}", path);
    let res = reqwest::blocking::get(&url).ok()?;
    let val: FGIntProperty = res.json().ok()?;
    Some(val.value)
}
