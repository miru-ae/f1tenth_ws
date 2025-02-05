use ackermann_msgs::msg::AckermannDriveStamped;
use anyhow::{Error, Result};
use rclrs::{self, Context};
use std::env;
use tokio::time::{interval, Duration};

#[tokio::main]
async fn main() -> Result<(), Error> {
    println!("This is test move node");
    let context = Context::new(env::args())?;
    let node = rclrs::create_node(&context, "test_drive")?;
    let publisher =
        node.create_publisher::<AckermannDriveStamped>("/drive", rclrs::QOS_PROFILE_DEFAULT)?;

    let mut msg = AckermannDriveStamped::default();

    // First phase: Set speed to 10.0 with 100ms interval
    let mut interval_fast = interval(Duration::from_millis(100));
    for _ in 0..100 {
        interval_fast.tick().await;
        msg.drive.speed = 5.0;
        println!("driving");
        let _ = publisher.publish(&msg);
    }

    // Second phase: Set speed to 0.0 with 10ms interval
    let mut interval_slow = interval(Duration::from_millis(10));
    for _ in 0..100 {
        interval_slow.tick().await;
        msg.drive.speed = 0.0;
        println!("stop");
        let _ = publisher.publish(&msg);
    }

    // First phase: Set speed to 10.0 with 100ms interval
    let mut interval_fast = interval(Duration::from_millis(100));
    for _ in 0..100 {
        interval_fast.tick().await;
        msg.drive.speed = 5.0;
        println!("driving");
        let _ = publisher.publish(&msg);
    }

    // Second phase: Set speed to 0.0 with 10ms interval
    let mut interval_slow = interval(Duration::from_millis(10));
    for _ in 0..100 {
        interval_slow.tick().await;
        msg.drive.speed = 0.0;
        println!("stop");
        let _ = publisher.publish(&msg);
    }

    Ok(())
}
