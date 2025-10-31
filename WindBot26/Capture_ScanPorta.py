import pyrealsense2 as rs

BAG_OUTPUT = "Bag_Scan_Porta.bag"
NUM_FRAMES = 20

pipeline = rs.pipeline()
config = rs.config()
config.enable_record_to_file(BAG_OUTPUT)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)  
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)  # Adiciona captura RGB

# Detect device and sensors
ctx = rs.context()
dev = ctx.query_devices()[0]

# Set parameters for depth and color sensors
for sensor in dev.query_sensors():
    if sensor.is_depth_sensor():
        # Depth sensor options
        try:
            sensor.set_option(rs.option.gain, 16)
            sensor.set_option(rs.option.laser_power, 150)
            sensor.set_option(rs.option.laser_state, 1)
            sensor.set_option(rs.option.enable_auto_exposure, 1)
            sensor.set_option(rs.option.exposure, 33000)
            sensor.set_option(rs.option.white_balance, 4600)
        except Exception as e:
            print(f"Depth sensor option error: {e}")
    elif sensor.get_info(rs.camera_info.name) == 'RGB Camera':
        # Color sensor options
        try:
            sensor.set_option(rs.option.enable_auto_exposure, 1)
            sensor.set_option(rs.option.exposure, 156)
            sensor.set_option(rs.option.brightness, 0)
            sensor.set_option(rs.option.contrast, 50)
            sensor.set_option(rs.option.saturation, 64)
            sensor.set_option(rs.option.sharpness, 50)
            sensor.set_option(rs.option.gain, 64)
            sensor.set_option(rs.option.gamma, 300)
            sensor.set_option(rs.option.white_balance, 4600)
            sensor.set_option(rs.option.backlight_compensation, 0)
            sensor.set_option(rs.option.hue, 0)
            sensor.set_option(rs.option.power_line_frequency, 3)
        except Exception as e:
            print(f"Color sensor option error: {e}")

spatial_magnitude = 2
spatial_smooth_alpha = 0.3  
spatial_smooth_delta = 30  
spatial_holes_fill = 1  

filter_spatial = rs.spatial_filter()
filter_spatial.set_option(rs.option.filter_magnitude, spatial_magnitude)
filter_spatial.set_option(rs.option.filter_smooth_alpha, spatial_smooth_alpha)
filter_spatial.set_option(rs.option.filter_smooth_delta, spatial_smooth_delta)
filter_spatial.set_option(rs.option.holes_fill, spatial_holes_fill)

filter_temporal = rs.temporal_filter(smooth_alpha=0.6, smooth_delta=50, persistence_control=3) 

pipeline.start(config)
print(f"Gravando para {BAG_OUTPUT}, capturando {NUM_FRAMES} frames...")

for i in range(NUM_FRAMES):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue
    depth_filtered = filter_spatial.process(depth_frame)
    depth_filtered = filter_temporal.process(depth_filtered)
    # Exemplo: captura a distância do centro em centímetros
    center_x = int(depth_frame.get_width() / 2)
    center_y = int(depth_frame.get_height() / 2)
    distance_m = depth_frame.get_distance(center_x, center_y)
    distance_cm = distance_m * 100
    print(f"Frame {i+1} capturado e filtrado. Distância centro: {distance_cm:.2f} cm")

pipeline.stop()
print("Gravação finalizada.")