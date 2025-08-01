#!/usr/bin/env python3
import json, paho.mqtt.client as mqtt

BROKER = "acbbe4c6e23f429bb880f91e1bea473c.s1.eu.hivemq.cloud"          # ⇐ change me
PORT   = 8883                  # ⇐ 8883 = MQTT over TLS
USER = "robot" 
PASS = "Password123" 
TOPIC  = "teleop/joint_states"



def on_msg(cli, userdata, msg):
    js = json.loads(msg.payload)
    stamp  = js["t_ns"]      # nanoseconds since epoch (publisher clock)
    q      = js["position"]  # 7-element joint vector
    print(f"[{stamp}] q = {q}")
    # TODO: feed q into teleoperation controller / ROS bridge here


cli = mqtt.Client(client_id="uk_subscriber")
cli.username_pw_set(USER, PASS)
cli.tls_set()
cli.on_message = on_msg
cli.connect(BROKER, PORT, keepalive=30)
cli.subscribe(TOPIC, qos=1)
cli.loop_forever()


