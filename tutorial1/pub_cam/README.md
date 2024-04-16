A simple node to publish video streams to a ros2 topic.


## Nodes
`cam_server`

### Publishers
  - `/cam/image`: Base topic, used for raw transport.
  - `/cam/image/compressed`: Topic for "compressed" transport.

### Subscribers
  - `/cam/param` (`std_msgs::msg::String`): enable (ON) / disable (OFF) the camera

