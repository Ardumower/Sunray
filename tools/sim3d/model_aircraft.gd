extends Node3D
# Parked electric model aircraft; dimensions in metres, nose toward local +Z.
# Primitive meshes keep the scene inexpensive on the Orange Pi GPU.
func ellipsoid(parent: Node3D, pos: Vector3, size: Vector3, color: Color):
 var node := MeshInstance3D.new()
 var mesh := SphereMesh.new()
 mesh.radius=.5
 mesh.height=1.0
 mesh.radial_segments=16
 mesh.rings=8
 node.mesh=mesh
 node.position=pos
 node.scale=size
 var material := StandardMaterial3D.new()
 material.albedo_color=color
 material.roughness=.65
 node.material_override=material
 parent.add_child(node)

func build(world: Node3D, station: Node3D, specs: Array):
 for spec in specs:
  var plane := Node3D.new()
  plane.name=spec.get("name","Model aircraft")
  add_child(plane)
  # Positive front_m is on the grass in front of the table entrances.
  plane.position=station.position+station.basis.x*float(spec.right_m)+station.basis.z*float(spec.front_m)
  plane.rotation.y=station.rotation.y+deg_to_rad(float(spec.get("heading_deg",0)))
  var span: float=spec.get("wingspan_m",1.8)
  var length: float=spec.get("length_m",1.4)
  var accent := Color.from_string(spec.get("color","#c92e27"),Color.RED)
  var white := Color(.91,.92,.89)
  ellipsoid(plane,Vector3(0,.29,0),Vector3(.18,.20,length),white)
  # Fuselage collision and thin wing collisions also reach the mower bumper.
  world.box(plane,Vector3(0,.27,0),Vector3(.13,.13,length*.85),white,true)
  world.box(plane,Vector3(0,.30,.09),Vector3(span,.045,.29),white,true)
  for side in [-1.0,1.0]:
   world.box(plane,Vector3(side*span*.42,.325,.09),Vector3(span*.16,.008,.29),accent)
   world.box(plane,Vector3(side*span*.27,.326,.09),Vector3(.065,.009,.29),accent)
  world.box(plane,Vector3(0,.31,-length*.38),Vector3(span*.35,.035,.19),accent,true)
  world.box(plane,Vector3(0,.43,-length*.36),Vector3(.035,.28,.22),accent,true)
  ellipsoid(plane,Vector3(0,.405,.17),Vector3(.125,.12,.32),Color(.09,.16,.21))
  ellipsoid(plane,Vector3(0,.29,length*.47),Vector3(.13,.13,.18),accent)
  # Stopped two-blade propeller, landing gear and tail wheel.
  var prop: Node3D = world.box(plane,Vector3(0,.29,length*.55),Vector3(.025,.40,.018),Color(.12,.12,.13))
  prop.rotation.z=.4
  world.box(plane,Vector3(0,.14,.18),Vector3(.42,.025,.025),Color(.21,.22,.23))
  for side in [-1.0,1.0]:
   world.box(plane,Vector3(side*.16,.21,.18),Vector3(.02,.17,.025),Color(.21,.22,.23))
   ellipsoid(plane,Vector3(side*.22,.10,.18),Vector3(.065,.18,.18),Color(.055,.055,.06))
  ellipsoid(plane,Vector3(0,.065,-length*.37),Vector3(.035,.10,.10),Color(.055,.055,.06))
