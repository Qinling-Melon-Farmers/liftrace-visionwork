    <!-- 由 generate_oblique_vehicle_sdf.py 注入基线机架；不要手工复制下视链。 -->
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
      <sensor name="aux_camera" type="depth">
        <always_on>true</always_on>
        <update_rate>20</update_rate>
        <visualize>true</visualize>
        <camera>
          <horizontal_fov>1.0471975512</horizontal_fov>
          <image>
            <width>640</width><height>480</height><format>R8G8B8</format>
          </image>
          <clip><near>0.10</near><far>12.0</far></clip>
        </camera>
        <plugin name="aux_camera_controller" filename="libgazebo_ros_openni_kinect.so">
          <robotNamespace>/</robotNamespace>
          <cameraName>aux_camera</cameraName>
          <alwaysOn>true</alwaysOn><updateRate>20</updateRate>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <pointCloudTopicName>depth/points</pointCloudTopicName>
          <frameName>aux_camera_optical_frame</frameName>
          <pointCloudCutoff>0.20</pointCloudCutoff>
          <pointCloudCutoffMax>10.0</pointCloudCutoffMax>
          <distortion_k1>0</distortion_k1><distortion_k2>0</distortion_k2>
          <distortion_k3>0</distortion_k3><distortion_t1>0</distortion_t1>
          <distortion_t2>0</distortion_t2>
        </plugin>
      </sensor>
    </link>
    <joint name="aux_camera_joint" type="fixed">
      <child>aux_camera_link</child>
      <parent>iris::base_link</parent>
    </joint>
