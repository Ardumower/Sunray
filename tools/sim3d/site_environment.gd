extends Node3D
# Approximate geometry traced from satellite imagery, with real camera references.
# Coordinates are east/north metres relative to the simulator Cloud map origin.
var materials := {}
func material(color: Color) -> StandardMaterial3D:
 var key := color.to_html()
 if not materials.has(key):
  var m := StandardMaterial3D.new()
  m.albedo_color=color
  m.roughness=1.0
  materials[key]=m
 return materials[key]

func surface(points: Array, color: Color, height: float, ground_material: Material = null):
 var polygon := PackedVector2Array()
 for p in points: polygon.append(Vector2(p.X,-p.Y))
 var indices := Geometry2D.triangulate_polygon(polygon)
 if indices.is_empty(): return
 var mesh := SurfaceTool.new()
 mesh.begin(Mesh.PRIMITIVE_TRIANGLES)
 for i in indices:
  mesh.set_normal(Vector3.UP)
  mesh.add_vertex(Vector3(polygon[i].x,height,polygon[i].y))
 var node := MeshInstance3D.new()
 node.mesh=mesh.commit()
 var m := material(color).duplicate()
 m.cull_mode=BaseMaterial3D.CULL_DISABLED
 node.material_override=ground_material if ground_material != null else m
 add_child(node)

func tree(world: Node3D, p: Vector2, height: float, radius: float, color: Color):
 world.box(self,Vector3(p.x,height*.28,-p.y),Vector3(.30,height*.56,.30),Color(.23,.18,.12),true)
 var crown := MeshInstance3D.new()
 var mesh := SphereMesh.new()
 mesh.radius=radius
 mesh.height=height*.65
 mesh.radial_segments=8
 mesh.rings=4
 crown.mesh=mesh
 crown.position=Vector3(p.x,height*.66,-p.y)
 crown.material_override=material(color)
 add_child(crown)

# Distant tree silhouettes use two instanced meshes per wood, without thousands
# of scene nodes, shadow casters or collision bodies outside the test field.
func distant_wood(area: Dictionary, rng: RandomNumberGenerator):
 var polygon := PackedVector2Array()
 var minimum := Vector2(INF,INF)
 var maximum := Vector2(-INF,-INF)
 for p in area.points:
  var v := Vector2(p.X,p.Y)
  polygon.append(v);minimum=minimum.min(v);maximum=maximum.max(v)
 var samples: Array[Vector3]=[]
 var spacing: float=area.get("spacing_m",14.0)
 var x := minimum.x
 while x<=maximum.x:
  var y := minimum.y
  while y<=maximum.y:
   var p := Vector2(x+rng.randf_range(-3,3),y+rng.randf_range(-3,3))
   if Geometry2D.is_point_in_polygon(p,polygon):
    samples.append(Vector3(p.x,rng.randf_range(area.min_height_m,area.max_height_m),-p.y))
   y+=spacing
  x+=spacing
 if samples.is_empty(): return
 var crowns := MultiMesh.new()
 crowns.transform_format=MultiMesh.TRANSFORM_3D
 crowns.use_colors=true
 var sphere := SphereMesh.new()
 sphere.radius=1.0;sphere.height=2.0;sphere.radial_segments=8;sphere.rings=3
 crowns.mesh=sphere;crowns.instance_count=samples.size()
 var trunks := MultiMesh.new()
 trunks.transform_format=MultiMesh.TRANSFORM_3D
 var trunk := BoxMesh.new()
 trunk.size=Vector3(.45,1,.45)
 trunks.mesh=trunk;trunks.instance_count=samples.size()
 for i in range(samples.size()):
  var p: Vector3=samples[i]
  var radius: float=float(area.get("crown_radius_m",8.0))*rng.randf_range(.85,1.15)
  crowns.set_instance_transform(i,Transform3D(Basis.IDENTITY.scaled(Vector3(radius,p.y*.34,radius)),Vector3(p.x,p.y*.67,p.z)))
  crowns.set_instance_color(i,Color(rng.randf_range(.80,1.05),rng.randf_range(.85,1.08),rng.randf_range(.8,1.05)))
  trunks.set_instance_transform(i,Transform3D(Basis.IDENTITY.scaled(Vector3(1,p.y*.55,1)),Vector3(p.x,p.y*.275,p.z)))
 for entry in [[crowns,Color(.12,.23,.10)],[trunks,Color(.23,.20,.15)]]:
  var visual := MultiMeshInstance3D.new()
  visual.multimesh=entry[0]
  var m := material(entry[1]).duplicate()
  m.vertex_color_use_as_albedo=true
  visual.material_override=m
  visual.cast_shadow=GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
  add_child(visual)
 print("SIM3D distant wood ",area.name,": ",samples.size()," trees")

func molehill(spec: Dictionary):
 var body := StaticBody3D.new()
 body.name = str(spec.id)
 body.position=Vector3(spec.X,.028,-spec.Y)
 body.set_meta("kind","molehill")
 body.rotation.y=deg_to_rad(float(spec.get("heading_deg",0.0)))
 add_child(body)
 var radius: float=spec.get("radius_m",.35)
 var half_length: float=float(spec.get("length_m",2.0*radius))/2.0
 var half_width: float=float(spec.get("width_m",2.0*radius))/2.0
 var height: float=spec.height_m
 var rng := RandomNumberGenerator.new()
 rng.seed=int(spec.get("seed",1))
 var rings: Array[PackedVector3Array]=[]
 const SIDES=18
 for ring in range(5):
  var vertices := PackedVector3Array()
  var fraction := float(ring)/4.0
  for i in range(SIDES):
   var angle := TAU*float(i)/SIDES
   var r := radius*fraction*rng.randf_range(.88,1.12)
   var h := height*pow(1.0-fraction,1.15)*rng.randf_range(.85,1.10)
   vertices.append(Vector3(cos(angle)*r*half_length/radius,h,sin(angle)*r*half_width/radius))
  rings.append(vertices)
 var surface_tool := SurfaceTool.new()
 surface_tool.begin(Mesh.PRIMITIVE_TRIANGLES)
 for ring in range(4):
  for i in range(SIDES):
   var j := (i+1)%SIDES
   for v in [rings[ring][i],rings[ring+1][j],rings[ring+1][i],rings[ring][i],rings[ring][j],rings[ring+1][j]]:
    var shade := rng.randf_range(.82,1.12)
    surface_tool.set_color(Color(.24*shade,.15*shade,.075*shade))
    surface_tool.add_vertex(v)
 surface_tool.generate_normals()
 var mesh := surface_tool.commit()
 var visual := MeshInstance3D.new()
 visual.mesh=mesh
 var soil := StandardMaterial3D.new()
 soil.vertex_color_use_as_albedo=true
 soil.roughness=1.0
 soil.cull_mode=BaseMaterial3D.CULL_DISABLED
 visual.material_override=soil
 body.add_child(visual)
 var collider := CollisionShape3D.new()
 collider.shape=mesh.create_convex_shape()
 body.add_child(collider)

func build(world: Node3D, cfg: Dictionary, map_data: Dictionary):
 for mound in cfg.get("molehills",[]): molehill(mound)
 # The large exclusion is the real runway: ground surface, not a raised wall.
 var asphalt := ShaderMaterial.new()
 asphalt.shader=preload("res://asphalt.gdshader")
 var grain := FastNoiseLite.new()
 grain.seed=1739
 grain.noise_type=FastNoiseLite.TYPE_SIMPLEX_SMOOTH
 grain.frequency=.08
 grain.fractal_octaves=3
 var aggregate := NoiseTexture2D.new()
 aggregate.width=512
 aggregate.height=512
 aggregate.seamless=true
 aggregate.generate_mipmaps=true
 aggregate.noise=grain
 asphalt.set_shader_parameter("aggregate",aggregate)
 for polygon in map_data.get("exclusions",[]):
  if polygon.size()>2: surface(polygon,Color(.42,.43,.44),.09,asphalt)
 for patch in cfg.get("ground_patches",[]):
  var ground_material: Material = null
  if patch.name != "track_south": ground_material=world.grass_material(Color(.89,.94,.81))
  surface(patch.points,Color(patch.color),.06,ground_material)
 for building in cfg.get("buildings",[]):
  var house := Node3D.new()
  add_child(house)
  house.position=Vector3(building.X,0,-building.Y)
  house.rotation.y=deg_to_rad(float(building.get("yaw_deg",0))) - PI/2
  var width: float=building.width_m
  var depth: float=building.depth_m
  var height: float=building.height_m
  world.box(house,Vector3(0,height/2,0),Vector3(width,height,depth),Color(.39,.35,.28),true)
  world.box(house,Vector3(0,height+.08,0),Vector3(width+.35,.16,depth+.35),Color(.17,.19,.19),true)
  world.box(house,Vector3(0,height*.43,depth/2+.015),Vector3(.85,height*.82,.025),Color(.23,.26,.24))
  for x in [-width*.32,width*.32]:
   world.box(house,Vector3(x,height*.60,depth/2+.03),Vector3(width*.19,.65,.03),Color(.64,.70,.68))
 var rng := RandomNumberGenerator.new()
 rng.seed=10282026
 for area in cfg.get("tree_areas",[]):
  if area.get("distant",false):
   distant_wood(area,rng)
   continue
  var polygon := PackedVector2Array()
  var minimum := Vector2(INF,INF)
  var maximum := Vector2(-INF,-INF)
  for p in area.points:
   var v := Vector2(p.X,p.Y)
   polygon.append(v)
   minimum=minimum.min(v);maximum=maximum.max(v)
  var spacing: float=area.get("spacing_m",9.0)
  var x := minimum.x
  while x<=maximum.x:
   var y := minimum.y
   while y<=maximum.y:
    var p := Vector2(x+rng.randf_range(-2,2),y+rng.randf_range(-2,2))
    if Geometry2D.is_point_in_polygon(p,polygon):
     var h := rng.randf_range(float(area.get("min_height_m",9)),float(area.get("max_height_m",16)))
     tree(world,p,h,rng.randf_range(2.8,4.6),Color(.10+rng.randf()*.08,.22+rng.randf()*.12,.07+rng.randf()*.06))
    y+=spacing
   x+=spacing
