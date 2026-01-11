#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleGlobalPosition, SensorGps


class GpsPositionDisplay(Node):
    """显示GPS坐标的节点。"""

    def __init__(self) -> None:
        super().__init__('gps_position_display')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        
        self.sensor_gps_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/sensor_gps', self.sensor_gps_callback, qos_profile)

        self.vehicle_global_position = VehicleGlobalPosition()
        self.sensor_gps = SensorGps()
        
        self.global_position_received = False
        self.sensor_gps_received = False
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_time = self.get_clock().now()
        
        self.get_logger().info("GPS坐标显示节点已启动，正在检查GPS状态...")

    def vehicle_global_position_callback(self, vehicle_global_position):
        self.global_position_received = True
        self.vehicle_global_position = vehicle_global_position
        
        if self.global_position_received and not hasattr(self, 'first_global_position_received'):
            self.first_global_position_received = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"✓ VehicleGlobalPosition话题已连接 (耗时: {elapsed:.2f}秒)")

    def sensor_gps_callback(self, sensor_gps):
        self.sensor_gps_received = True
        self.sensor_gps = sensor_gps
        
        if self.sensor_gps_received and not hasattr(self, 'first_sensor_gps_received'):
            self.first_sensor_gps_received = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f"✓ SensorGps话题已连接 (耗时: {elapsed:.2f}秒)")

    def timer_callback(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"GPS状态报告 (运行时间: {elapsed:.1f}秒)")
        self.get_logger().info("=" * 60)
        
        # 检查话题连接状态
        self.get_logger().info(f"VehicleGlobalPosition话题: {'✓ 已连接' if self.global_position_received else '✗ 未连接'}")
        self.get_logger().info(f"SensorGps话题: {'✓ 已连接' if self.sensor_gps_received else '✗ 未连接'}")
        
        # 显示融合后的全球位置信息
        if self.global_position_received:
            lat = self.vehicle_global_position.lat
            lon = self.vehicle_global_position.lon
            alt = self.vehicle_global_position.alt
            alt_ellipsoid = self.vehicle_global_position.alt_ellipsoid
            
            self.get_logger().info("融合后的GPS坐标 (VehicleGlobalPosition):")
            self.get_logger().info(f"  纬度: {lat:.8f}°")
            self.get_logger().info(f"  经度: {lon:.8f}°")
            self.get_logger().info(f"  高度 (AMSL): {alt:.2f}米")
            self.get_logger().info(f"  高度 (椭球): {alt_ellipsoid:.2f}米")
            
            # 显示有效性信息
            lat_lon_valid = self.vehicle_global_position.lat_lon_valid
            alt_valid = self.vehicle_global_position.alt_valid
            self.get_logger().info(f"  坐标有效性: 纬度经度={'有效' if lat_lon_valid else '无效'}, 高度={'有效' if alt_valid else '无效'}")
            
            # 显示精度信息
            eph = self.vehicle_global_position.eph
            epv = self.vehicle_global_position.epv
            self.get_logger().info(f"  精度: 水平={eph:.2f}米, 垂直={epv:.2f}米")
            
            # 显示地形高度
            terrain_alt = self.vehicle_global_position.terrain_alt
            terrain_alt_valid = self.vehicle_global_position.terrain_alt_valid
            if terrain_alt_valid:
                self.get_logger().info(f"  地形高度: {terrain_alt:.2f}米")
            
            # 显示是否使用了航位推算
            dead_reckoning = self.vehicle_global_position.dead_reckoning
            if dead_reckoning:
                self.get_logger().warn("  ⚠ 使用了航位推算，位置可能不够准确")
        
        # 显示原始GPS传感器数据
        if self.sensor_gps_received:
            latitude_deg = self.sensor_gps.latitude_deg
            longitude_deg = self.sensor_gps.longitude_deg
            altitude_msl_m = self.sensor_gps.altitude_msl_m
            altitude_ellipsoid_m = self.sensor_gps.altitude_ellipsoid_m
            
            self.get_logger().info("原始GPS传感器数据 (SensorGps):")
            self.get_logger().info(f"  纬度: {latitude_deg:.8f}°")
            self.get_logger().info(f"  经度: {longitude_deg:.8f}°")
            self.get_logger().info(f"  高度 (AMSL): {altitude_msl_m:.2f}米")
            self.get_logger().info(f"  高度 (椭球): {altitude_ellipsoid_m:.2f}米")
            
            # 显示GPS状态
            fix_type = self.sensor_gps.fix_type
            fix_type_str = self.get_fix_type_string(fix_type)
            satellites_used = self.sensor_gps.satellites_used
            
            self.get_logger().info(f"  GPS状态: 定位类型={fix_type_str}, 卫星数量={satellites_used}")
            
            # 显示精度信息
            eph = self.sensor_gps.eph
            epv = self.sensor_gps.epv
            hdop = self.sensor_gps.hdop
            vdop = self.sensor_gps.vdop
            
            self.get_logger().info(f"  精度: 水平={eph:.2f}米, 垂直={epv:.2f}米")
            self.get_logger().info(f"  精度因子: HDOP={hdop:.2f}, VDOP={vdop:.2f}")
            
            # 显示速度信息
            vel_m_s = self.sensor_gps.vel_m_s
            vel_n_m_s = self.sensor_gps.vel_n_m_s
            vel_e_m_s = self.sensor_gps.vel_e_m_s
            vel_d_m_s = self.sensor_gps.vel_d_m_s
            
            if self.sensor_gps.vel_ned_valid:
                self.get_logger().info(f"  速度: 地面速度={vel_m_s:.2f}米/秒")
                self.get_logger().info(f"  速度分量: 北={vel_n_m_s:.2f}, 东={vel_e_m_s:.2f}, 下={vel_d_m_s:.2f}米/秒")
            
            # 显示UTC时间
            time_utc_usec = self.sensor_gps.time_utc_usec
            if time_utc_usec > 0:
                import datetime
                utc_time = datetime.datetime.utcfromtimestamp(time_utc_usec / 1e6)
                self.get_logger().info(f"  UTC时间: {utc_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            
            # 显示干扰和欺骗状态
            jamming_state = self.sensor_gps.jamming_state
            spoofing_state = self.sensor_gps.spoofing_state
            
            jamming_str = self.get_jamming_state_string(jamming_state)
            spoofing_str = self.get_spoofing_state_string(spoofing_state)
            
            self.get_logger().info(f"  干扰状态: {jamming_str}")
            self.get_logger().info(f"  欺骗状态: {spoofing_str}")

        # 诊断建议
        self.get_logger().info("=" * 60)
        self.get_logger().info("诊断建议:")
        
        if not self.global_position_received and not self.sensor_gps_received:
            self.get_logger().error("✗ 所有GPS相关话题都未连接！")
            self.get_logger().error("  请检查:")
            self.get_logger().error("  1. GPS模块是否正确连接")
            self.get_logger().error("  2. GPS天线是否正确安装")
            self.get_logger().error("  3. 户外环境是否有良好的卫星信号")
            self.get_logger().error("  4. PX4飞控是否正确配置了GPS参数")
        elif not self.global_position_received:
            self.get_logger().warn("⚠ VehicleGlobalPosition话题未连接")
            self.get_logger().warn("  可能原因: 位置估计器尚未初始化或GPS信号不足")
        elif not self.sensor_gps_received:
            # 检查VehicleGlobalPosition是否有效
            if hasattr(self.vehicle_global_position, 'lat_lon_valid') and self.vehicle_global_position.lat_lon_valid:
                self.get_logger().info("✓ 虽然SensorGps话题未连接，但VehicleGlobalPosition提供了有效的GPS坐标")
                self.get_logger().info("  这是正常的，因为VehicleGlobalPosition包含了融合后的位置信息")
            else:
                self.get_logger().warn("⚠ SensorGps话题未连接")
                self.get_logger().warn("  可能原因: GPS模块未正确连接或未上电")
        elif self.sensor_gps_received:
            # 检查GPS定位类型
            fix_type = self.sensor_gps.fix_type
            if fix_type < 3:  # 3D定位
                self.get_logger().warn("⚠ GPS定位类型不足，当前为2D或无定位")
                self.get_logger().warn("  请确保在户外环境中，GPS天线有良好的视野")
            if self.sensor_gps.satellites_used < 6:
                self.get_logger().warn(f"⚠ 卫星数量较少 ({self.sensor_gps.satellites_used}颗)")
                self.get_logger().warn("  可能影响定位精度，请确保GPS天线有良好的视野")
        
        self.get_logger().info("=" * 60)

    def get_fix_type_string(self, fix_type):
        """获取GPS定位类型的字符串表示。"""
        fix_type_map = {
            0: "无定位",
            1: "无定位",
            2: "2D定位",
            3: "3D定位",
            4: "RTCM码差分",
            5: "RTK浮点解",
            6: "RTK固定解",
            8: "外推定位"
        }
        return fix_type_map.get(fix_type, f"未知 ({fix_type})")

    def get_jamming_state_string(self, jamming_state):
        """获取GPS干扰状态的字符串表示。"""
        jamming_state_map = {
            0: "未知",
            1: "正常",
            2: "已缓解",
            3: "已检测到"
        }
        return jamming_state_map.get(jamming_state, f"未知 ({jamming_state})")

    def get_spoofing_state_string(self, spoofing_state):
        """获取GPS欺骗状态的字符串表示。"""
        spoofing_state_map = {
            0: "未知",
            1: "正常",
            2: "已缓解",
            3: "已检测到"
        }
        return spoofing_state_map.get(spoofing_state, f"未知 ({spoofing_state})")


def main(args=None) -> None:
    print('启动GPS坐标显示节点...')
    rclpy.init(args=args)
    gps_display = GpsPositionDisplay()
    rclpy.spin(gps_display)
    gps_display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)