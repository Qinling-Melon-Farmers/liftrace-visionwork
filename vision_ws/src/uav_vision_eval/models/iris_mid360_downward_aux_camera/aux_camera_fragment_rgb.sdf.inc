    <!-- 单目评测使用轻量 RGB 插件；避免把未使用的深度图和点云开销计入单目路线。 -->
    <link name="aux_camera_link">
      <pose>0.12 0 0.00 0 {pitch_rad} 0</pose>
      <inertial>
        <mass>0.056</mass>
        <inertia>
          <ixx>4.1e-5</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>5.9e-6</iyy><iyz>0</iyz><izz>5.9e-6</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.09 0.025 0.025</size></box></geometry>
        <material>
          <ambient>0.08 0.08 0.08 1</ambient>
          <diffuse>0.12 0.12 0.12 1</diffuse>
        </material>
      </visual>
      <sensor name="aux_camera" type="camera">
        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.0471975512</horizontal_fov>
          <image>
            <width>640</width><height>480</height><format>R8G8B8</format>
          </image>
          <clip><near>0.10</near><far>12.0</far></clip>
        </camera>
        <plugin name="aux_camera_controller" filename="libgazebo_ros_camera.so">
          <robotNamespace>/</robotNamespace>
          <cameraName>aux_camera</cameraName>
          <alwaysOn>true</alwaysOn><updateRate>20</updateRate>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>aux_camera_optical_frame</frameName>
          <distortionK1>0</distortionK1><distortionK2>0</distortionK2>
          <distortionK3>0</distortionK3><distortionT1>0</distortionT1>
          <distortionT2>0</distortionT2>
        </plugin>
      </sensor>
    </link>
    <joint name="aux_camera_joint" type="fixed">
      <child>aux_camera_link</child>
      <parent>iris::base_link</parent>
    </joint>
