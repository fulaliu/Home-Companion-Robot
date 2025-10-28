#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from megapi import MegaPi
import time
import sys
import argparse

# 电机端口映射
MFR = 2
MBL = 3
MBR = 10
MFL = 11


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR
        self.mbl = MBL
        self.mbr = MBR
        self.mfl = MFL

    def printConfiguration(self):
        print(f'MegaPiController: Port={self.port}, Motors: {MFR},{MBL},{MBR},{MFL}')

    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print(f"Set Motors: vfl:{int(vfl)} vfr:{int(vfr)} vbl:{int(vbl)} vbr:{int(vbr)}")
        self.bot.motorRun(self.mfl, int(vfl))
        self.bot.motorRun(self.mfr, int(vfr))
        self.bot.motorRun(self.mbl, int(vbl))
        self.bot.motorRun(self.mbr, int(vbr))

    def carStop(self):
        self.setFourMotors(0, 0, 0, 0)

    def carMixed(self, v_straight, v_rotate, v_slide):
        self.setFourMotors(
            v_rotate - v_straight + v_slide,
            v_rotate + v_straight + v_slide,
            v_rotate - v_straight - v_slide,
            v_rotate + v_straight - v_slide
        )

    def close(self):
        self.carStop()
        self.bot.close()
        self.bot.exit()


class MbotTwistNode(Node):
    def __init__(self, controller: MegaPiController):
        super().__init__('mbot_controller')
        self.ctrl = controller

        # 参数
        self.declare_parameter('max_speed', 120)
        self.declare_parameter('deadband', 0.02)
        self.declare_parameter('cmd_topic', '/cmd_vel')
        #self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('cmd_timeout', 3)
        self.declare_parameter('sign_vx', 1)
        self.declare_parameter('sign_vy', 1)
        self.declare_parameter('sign_wz', 1)

        self.max_speed = self.get_parameter('max_speed').value
        self.deadband = self.get_parameter('deadband').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.sign_vx = self.get_parameter('sign_vx').value
        self.sign_vy = self.get_parameter('sign_vy').value
        self.sign_wz = self.get_parameter('sign_wz').value

        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, self.cmd_topic, self.twist_callback, 10)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(f"订阅 {self.cmd_topic}, max_speed={self.max_speed}, timeout={self.cmd_timeout}")

    def twist_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        vx = self.apply_deadband(msg.linear.x) * self.sign_vx
        vy = self.apply_deadband(msg.linear.y) * self.sign_vy
        wz = self.apply_deadband(msg.angular.z) * self.sign_wz

        v_straight = self.clamp(vx * self.max_speed)
        v_slide = self.clamp(vy * self.max_speed)
        v_rotate = self.clamp(wz * self.max_speed)

        self.ctrl.carMixed(v_straight, v_rotate, v_slide)

    def apply_deadband(self, v):
        return 0.0 if abs(v) < self.deadband else v
    def clamp(self, v):
        return max(-self.max_speed, min(self.max_speed, v))

    def watchdog(self):
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.ctrl.carStop()

    def destroy_node(self):
        self.ctrl.carStop()
        super().destroy_node()


def test_mode(ctrl: MegaPiController):
    print("进入测试模式...")
    ctrl.carMixed(60, 0, 0)  # 前进
    time.sleep(1.5)
    ctrl.carStop()
    time.sleep(0.5)
    ctrl.carMixed(0, 50, 0)  # 旋转
    time.sleep(1.5)
    ctrl.carStop()
    time.sleep(0.5)
    ctrl.carMixed(0, 0, 50)  # 平移
    time.sleep(1.5)
    ctrl.carStop()
    print("测试结束")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', action='store_true', help='测试模式')
    args = parser.parse_args()

    ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)

    if args.test:
        test_mode(ctrl)
    else:
        rclpy.init()
        node = MbotTwistNode(ctrl)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            ctrl.close()


if __name__ == '__main__':
    main()
