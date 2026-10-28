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

func surface(points: Array, color: Color, height: float, grass: Material = null):
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
 node.material_override=grass if grass != null else m
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

func molehill(spec: Dictionary):
 var body := StaticBody3D.new()
 body.name = str(spec.id)
 body.position=Vector3(spec.X,.028,-spec.Y)
 body.set_meta("kind","molehill")
 add_child(body)
 var radius: float=spec.radius_m
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
   vertices.append(Vector3(cos(angle)*r,h,sin(angle)*r))
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
 for polygon in map_data.get("exclusions",[]):
  if polygon.size()>2: surface(polygon,Color(.35,.37,.23),.09,world.grass_material(Color(.85,.83,.61)))
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
