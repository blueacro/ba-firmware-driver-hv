use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, PartialEq)]
pub struct TimeIsNow {
    seconds: u8,
    minutes: u8,
    hours: u8
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct SetTime {
    seconds: u8,
    minutes: u8,
    hours: u8,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub enum Command {
    SetTime(SetTime),
    QueryTime,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub enum Response {
    TimeIsNow(TimeIsNow)
}
