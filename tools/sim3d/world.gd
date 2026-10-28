extends Node3D
# World axes: Godot (X,Y,Z) = Sunray (east, height, -north).
# Godot owns the true pose. No estimator/filtered GPS coordinates enter rendering.
var server := UDPServer.new()
var peer: PacketPeerUDP
var grass_texture: ImageTexture
var cfg: Dictionary
var map_data: Dictionary
var robot: CharacterBody3D
var camera: Camera3D
var dock := Vector2.ZERO
var dock_yaw := 0.0
var yaw := 0.0
var left := 0.0
var right := 0.0
var wheelbase := 0.48
var ticks_per_meter := 1100.0 / (PI * 0.26)
var enc_left := 0.0
var enc_right := 0.0
var flags := 0
var last_command := -10000
var sequence := 0
var last_reset := 0
var last_obstacle := 0
var contact := false
var bump := false
var linear := 0.0
var angular := 0.0
var dynamic_obstacle: StaticBody3D
var frame_seq := 0
var frame_busy := false
var next_frame := 0
var next_state := 0
var next_status := 0
var runtime := "/run/sunray-sim3d"

func read_json(path: String) -> Dictionary:
 var file := FileAccess.open(path, FileAccess.READ)
 if file == null: return {}
 var data = JSON.parse_string(file.get_as_text())
 return data if data is Dictionary else {}

func box(parent: Node3D, pos: Vector3, size: Vector3, color: Color, solid: bool = false) -> Node3D:
 var node: Node3D = StaticBody3D.new() if solid else Node3D.new()
 parent.add_child(node)
 node.position = pos
 var mesh := MeshInstance3D.new()
 var shape := BoxMesh.new()
 shape.size = size
 mesh.mesh = shape
 var material := StandardMaterial3D.new()
 material.albedo_color = color
 material.roughness = 0.9
 mesh.material_override = material
 node.add_child(mesh)
 if solid:
  var collision := CollisionShape3D.new()
  var bounds := BoxShape3D.new()
  bounds.size = size
  collision.shape = bounds
  node.add_child(collision)
 return node

func point(p: Dictionary, height: float = 0.0) -> Vector3:
 return Vector3(float(p.X), height, -float(p.Y))

# Net centerlines are separate from mowing boundaries: exclusion polygons do not
# necessarily represent physical fences. Mesh is visual; collision is continuous.
func add_net(spec: Dictionary):
 var points: Array = spec.get("points", [])
 if points.size() < 2: return
 var height: float = spec.get("height_m", 3.0)
 var spacing: float = spec.get("post_spacing_m", 3.0)
 var post_width: float = spec.get("post_width_m", .06)
 var next_post := 0.0
 var travelled := 0.0
 var explicit_posts: Array = spec.get("post_points",[])
 for i in range(points.size()-1):
  var a := point(points[i], height/2)
  var b := point(points[i+1], height/2)
  var length := a.distance_to(b)
  if length < .01: continue
  var rolled_up: bool = spec.get("rolled_up",false)
  if rolled_up:
   # Rolled nets do not create an invisible full-height collision wall.
   var roll := box(self,(a+b)/2+Vector3(0,height/2,0),Vector3(.12,.12,length),Color(.13,.19,.14),true)
   roll.look_at(b+Vector3(0,height/2,0))
  var panel := StaticBody3D.new()
  add_child(panel)
  panel.position = (a+b)/2
  panel.look_at(b)
  var collision := CollisionShape3D.new()
  var bounds := BoxShape3D.new()
  bounds.size = Vector3(.025,height,length)
  collision.shape = bounds
  if not rolled_up: panel.add_child(collision)
  else: collision.free()
  var visual := MeshInstance3D.new()
  var quad := QuadMesh.new()
  quad.size = Vector2(length,height)
  visual.mesh = quad
  visual.rotation.y = PI/2
  var material := ShaderMaterial.new()
  material.shader = preload("res://net.gdshader")
  material.set_shader_parameter("panel_size",Vector2(length,height))
  material.set_shader_parameter("mesh_size",float(spec.get("mesh_size_m",.08)))
  material.set_shader_parameter("cord_width",float(spec.get("cord_width_m",.004)))
  visual.material_override = material
  if not rolled_up: panel.add_child(visual)
  else: visual.free()
  while next_post <= travelled+length:
   var pos := a.lerp(b,(next_post-travelled)/length)
   var near_known := false
   for known in explicit_posts:
    if pos.distance_to(point(known,height/2)) < spacing*.35: near_known=true
   if not near_known:
    box(self,pos,Vector3(post_width,height+.1,post_width),Color(.24,.26,.25),true)
   next_post += maxf(.1,spacing)
  travelled += length
 for known in explicit_posts:
  box(self,point(known,height/2),Vector3(post_width,height+.1,post_width),Color(.24,.26,.25),true)

func grass_material(color: Color = Color.WHITE) -> Material:
 if grass_texture == null:
  var fallback := StandardMaterial3D.new()
  fallback.albedo_color=Color(.22,.34,.12)*color
  return fallback
 var m := ShaderMaterial.new()
 m.shader=preload("res://grass.gdshader")
 m.set_shader_parameter("grass_albedo",grass_texture)
 m.set_shader_parameter("tint",color)
 m.set_shader_parameter("tile_metres",float(cfg.get("grass",{}).get("tile_metres",1.0)))
 return m

func _ready():
 cfg = read_json("res://scene.json")
 map_data = read_json("res://map.json")
 if cfg.is_empty() or map_data.get("dockpoints", []).size() < 2:
  push_error("scene.json and map.json with a dock path are required")
  get_tree().quit(2)
  return
 runtime = OS.get_environment("SUNRAY_SIM3D_RUNTIME") if OS.has_environment("SUNRAY_SIM3D_RUNTIME") else runtime
 DirAccess.make_dir_recursive_absolute(runtime)
 var points: Array = map_data.dockpoints
 var last: Dictionary = points[-1]
 var previous: Dictionary = points[-2]
 dock = Vector2(last.X,last.Y)
 dock_yaw = atan2(last.Y-previous.Y,last.X-previous.X)
 var grass_image := Image.load_from_file("res://assets/grass_albedo.png")
 if grass_image != null and not grass_image.is_empty():
  grass_image.generate_mipmaps()
  grass_texture=ImageTexture.create_from_image(grass_image)
 # Large simple terrain; exact perimeter/exclusions come from the Cloud map.
 var ground := box(self,Vector3(0,-0.15,0),Vector3(600,0.3,600),Color(.19,.35,.09),true)
 (ground.get_child(0) as MeshInstance3D).material_override=grass_material(Color(.86,.92,.79))
 var perimeter := PackedVector2Array()
 for p in map_data.perimeter: perimeter.append(Vector2(p.X,-p.Y))
 var indices := Geometry2D.triangulate_polygon(perimeter)
 var surface := SurfaceTool.new()
 surface.begin(Mesh.PRIMITIVE_TRIANGLES)
 for i in indices:
  surface.set_normal(Vector3.UP)
  surface.add_vertex(Vector3(perimeter[i].x,.025,perimeter[i].y))
 if not indices.is_empty():
  var lawn := MeshInstance3D.new()
  lawn.mesh = surface.commit()
  lawn.material_override = grass_material()
  add_child(lawn)
 var surroundings := preload("res://site_environment.gd").new()
 add_child(surroundings)
 surroundings.build(self,cfg,map_data)
 for net in cfg.get("nets", []): add_net(net)
 var station := Node3D.new()
 add_child(station)
 var offset: float = cfg.camera.forward_m + cfg.dock.camera_gap_m
 station.position = Vector3(dock.x+cos(dock_yaw)*offset,0,-dock.y-sin(dock_yaw)*offset)
 station.rotation.y = dock_yaw-PI/2
 box(station,Vector3(0,cfg.dock.panel_height_m/2,0),Vector3(cfg.dock.panel_width_m,cfg.dock.panel_height_m,.06),Color(.08,.085,.08),true)
 # Dock table: marker panel at the far end, entrance toward local +Z.
 var table_w: float = cfg.dock.table_width_m
 var table_l: float = cfg.dock.table_length_m
 var table_h: float = cfg.dock.table_height_m
 var leg: float = cfg.dock.table_leg_m
 box(station,Vector3(0,table_h,table_l/2),Vector3(table_w,.06,table_l),Color(.42,.44,.43),true)
 for side in [-1.0,1.0]:
  box(station,Vector3(side*(table_w-leg)/2,table_h/2,table_l/2),Vector3(leg,table_h,table_l),Color(.30,.31,.32),true)
  for along in [leg/2,table_l-leg/2]:
   box(station,Vector3(side*(table_w-leg)/2,table_h/2,along),Vector3(leg,table_h,leg),Color(.20,.21,.22),true)
 # Additional preparation tables, to the right when looking toward the marker.
 for spec in cfg.get("extra_tables",[]):
  var table := Node3D.new()
  add_child(table)
  table.rotation.y = dock_yaw-PI/2
  table.position = station.position + station.basis.x*float(spec.right_m) - station.basis.z*float(spec.forward_m)
  var w: float=spec.width_m
  var l: float=spec.length_m
  var h: float=spec.height_m
  box(table,Vector3(0,h,l/2),Vector3(w,.06,l),Color(.48,.50,.48),true)
  for side in [-1.0,1.0]:
   for along in [.04,l-.04]:
    box(table,Vector3(side*(w-.08)/2,h/2,along),Vector3(.08,h,.08),Color(.36,.38,.38),true)
 # Concrete slabs visible beneath the real charging table.
 box(station,Vector3(0,.015,table_l/2),Vector3(table_w+.6,.03,table_l+.8),Color(.45,.46,.42))
 for x in [-float(cfg.dock.strip_spacing_m),0.0,float(cfg.dock.strip_spacing_m)]:
  box(station,Vector3(x,cfg.dock.strip_center_height_m,.035),Vector3(cfg.dock.strip_width_m,cfg.dock.strip_height_m,.012),Color(.98,.98,.98))
 robot = CharacterBody3D.new()
 add_child(robot)
 var col := CollisionShape3D.new()
 var bounds := BoxShape3D.new()
 bounds.size = Vector3(cfg.robot.width_m,.4,cfg.robot.length_m)
 col.shape = bounds
 col.position.y = .22
 robot.add_child(col)
 # The simulated robot housing is omitted from its own camera view.
 camera = Camera3D.new()
 robot.add_child(camera)
 camera.position = Vector3(0,cfg.camera.height_m,-cfg.camera.forward_m)
 camera.keep_aspect = Camera3D.KEEP_WIDTH # Fix horizontal FOV; vertical FOV follows aspect ratio
 camera.fov = cfg.camera.horizontal_fov_deg
 camera.rotation.x = deg_to_rad(-float(cfg.camera.downward_pitch_deg))
 camera.near = .1
 camera.far = 350
 camera.current = true
 yaw = dock_yaw
 var distance: float = cfg.spawn.distance_before_dock_m
 robot.position = Vector3(dock.x-cos(yaw)*distance,0,-dock.y+sin(yaw)*distance)
 robot.rotation.y = yaw-PI/2
 var sun := DirectionalLight3D.new()
 sun.rotation_degrees = Vector3(cfg.lighting.sun_elevation_deg,cfg.lighting.sun_azimuth_deg,0)
 sun.light_energy = cfg.lighting.energy
 sun.shadow_enabled = true
 add_child(sun)
 var env := WorldEnvironment.new()
 env.environment = Environment.new()
 env.environment.background_mode = Environment.BG_COLOR
 env.environment.background_color = Color(.48,.64,.8)
 env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
 env.environment.ambient_light_color = Color.WHITE
 env.environment.ambient_light_energy = .6
 add_child(env)
 if server.listen(19091,"127.0.0.1") != OK:
  push_error("Simulator UDP port 19091 unavailable")
  get_tree().quit(2)
 print("SIM3D ready; renderer=",RenderingServer.get_video_adapter_name()," map=",cfg.map_name)

func accept_commands():
 server.poll()
 while server.is_connection_available():
  peer = server.take_connection()
  last_command = -10000
  last_reset = 0
  sequence = 0
 if peer == null: return
 while peer.get_available_packet_count()>0:
  var parts := peer.get_packet().get_string_from_utf8().split(" ",false)
  if parts.size()!=16 or parts[0]!="S3D1": continue
  var valid := true
  for i in range(1,16):
   if not parts[i].is_valid_float() or not is_finite(float(parts[i])): valid=false
  if not valid: continue
  var seq := int(parts[1])
  if seq<=sequence: continue
  sequence=seq
  left=clampf(float(parts[2]),-.7,.7)
  right=clampf(float(parts[3]),-.7,.7)
  wheelbase=clampf(float(parts[5]),.1,2.0)
  ticks_per_meter=clampf(float(parts[6]),1,100000)
  flags=int(parts[7])
  last_command=Time.get_ticks_msec()
  var reset:=int(parts[8])
  if reset!=last_reset and reset>0:
   robot.position=Vector3(float(parts[9]),0,-float(parts[10]))
   yaw=float(parts[11])
   last_reset=reset
  var obstacle:=int(parts[12])
  if obstacle!=last_obstacle:
   if is_instance_valid(dynamic_obstacle): dynamic_obstacle.queue_free()
   var radius:=clampf(float(parts[15]),0,10)
   if radius>0:
    dynamic_obstacle=box(self,Vector3(float(parts[13]),.5,-float(parts[14])),Vector3(radius*2,1,radius*2),Color(.6,.15,.1),true)
   last_obstacle=obstacle

func _physics_process(delta):
 if robot==null:return
 accept_commands()
 if Time.get_ticks_msec()-last_command>500: left=0;right=0
 var speed: float=(left+right)/2
 var turn: float=(right-left)/wheelbase
 enc_left+=abs(left)*delta*ticks_per_meter
 enc_right+=abs(right)*delta*ticks_per_meter
 var before:=robot.position
 var old_yaw:=yaw
 if (flags&1)==0:
  if (flags&2)==0:yaw=wrapf(yaw+turn*delta,-PI,PI)
  robot.rotation.y=yaw-PI/2
  var collision:=robot.move_and_collide(Vector3(cos(yaw),0,-sin(yaw))*speed*delta)
  bump=collision!=null
 else:bump=false
 linear=robot.position.distance_to(before)/delta*sign(speed)
 angular=angle_difference(old_yaw,yaw)/delta
 contact=Vector2(robot.position.x,-robot.position.z).distance_to(dock)<.12 and abs(angle_difference(yaw,dock_yaw))<.25
 if peer!=null and Time.get_ticks_msec()>=next_state:
  var msg="S3D1 %d %.8f %.8f %.8f %.3f %.3f %.6f %.6f %d %d 0 0" % [sequence,robot.position.x,-robot.position.z,yaw,enc_left,enc_right,linear,angular,int(bump),int(contact)]
  peer.put_packet(msg.to_utf8_buffer())
  next_state=Time.get_ticks_msec()+40
 if Time.get_ticks_msec()>=next_status:
  var status={"x":robot.position.x,"y":-robot.position.z,"yaw":yaw,"linear":linear,"angular":angular,"charging":contact,"bumper":bump,"command_age_ms":Time.get_ticks_msec()-last_command,"frame":frame_seq,"adapter":RenderingServer.get_video_adapter_name(),"camera_calibrated":false}
  var f:=FileAccess.open(runtime+"/status.json.tmp",FileAccess.WRITE)
  if f:
   f.store_string(JSON.stringify(status));f.close()
   DirAccess.rename_absolute(runtime+"/status.json.tmp",runtime+"/status.json")
  next_status=Time.get_ticks_msec()+500

func _process(_delta):
 if camera==null or frame_busy or Time.get_ticks_msec()<next_frame:return
 frame_busy=true
 next_frame=Time.get_ticks_msec()+int(1000.0/float(cfg.camera.fps))
 var stamp:=int(Time.get_unix_time_from_system()*1000)
 await RenderingServer.frame_post_draw
 var image:=get_viewport().get_texture().get_image()
 if image!=null and not image.is_empty():
  var jpeg:=image.save_jpg_to_buffer(.9)
  frame_seq+=1
  var file:=FileAccess.open(runtime+"/camera.frame.tmp",FileAccess.WRITE)
  if file:
   file.store_buffer("SRF1".to_ascii_buffer())
   file.store_64(frame_seq)
   file.store_64(stamp)
   file.store_buffer(jpeg)
   file.close()
   DirAccess.rename_absolute(runtime+"/camera.frame.tmp",runtime+"/camera.frame")
 frame_busy=false
