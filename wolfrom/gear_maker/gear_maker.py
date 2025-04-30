"""
Gear Maker Plugin/Add-in for Autodesk CAD Software
This specific plugin calls the Fusion 360 API to create profile shifted spur gears.
Mainly used in ModLab DOE Project to streamline design process.

This plugin is a customized version built on top of GF-Gear-Generator by Juan Gras.
Many of the functions are refactored and renamed for better readability and organization.
Image icon credit to the original author of GF-Gear-Generator.

Usage:
    - Go to Utilities > Add-ins (Top of the screen)
    - Select the "Add-ins" tab, click the green "+" button and find the file path of this code
    - Alternatively, do "Create" and use the same file location
    - Run the add-in, optional: select "Run on startup"
    - The "Gear Generator" should show up in the tab
"""

import adsk.core
import adsk.fusion
import traceback
import math

defaultfc = True
handlers = []

def linear_space(start, end, divisions):
    step = (end - start) / (divisions - 1)
    return [start + step * i for i in range(divisions)]

def radians_to_degrees(radians):
    return radians * 180 / math.pi

def create_timeline_group(num_operations):
    app = adsk.core.Application.get()
    design = app.activeProduct
    timeline_groups = design.timeline.timelineGroups
    index = max(0, design.timeline.count - num_operations)
    timeline_groups.add(index, design.timeline.count - 1)

def calculate_gear_parameters(module, teeth, pressure_angle, helix_angle, gear_width, is_ring_gear, profile_shift, fast_compute, normal_system=False):
    """
    Calculate gear parameters and tooth profile coordinates.
    
    Args:
        module: Module of the gear
        teeth: Number of teeth
        pressure_angle: Pressure angle in radians
        helix_angle: Helix angle in radians
        gear_width: Width of the gear
        is_ring_gear: Boolean, True if it's a ring gear
        profile_shift: Profile shift coefficient
        fast_compute: Boolean, True for faster but less accurate calculation
        normal_system: Boolean, True for normal system calculation, False for transverse
    
    Returns:
        Tuple with various gear parameters and coordinates for the tooth profile
    """
    # Calculate modified module and pressure angle for normal system
    modified_module = module
    modified_pressure_angle = pressure_angle
    
    if normal_system:
        modified_module = module / math.cos(helix_angle)
        modified_pressure_angle = math.atan(math.tan(pressure_angle) / math.cos(helix_angle))
    
    # Basic gear dimensions
    pitch_diameter = modified_module * teeth
    pitch_radius = pitch_diameter / 2
    base_diameter = pitch_diameter * math.cos(modified_pressure_angle)
    base_radius = base_diameter / 2
    addendum_diameter = pitch_diameter + 2 * module
    addendum_radius = addendum_diameter / 2
    dedendum_diameter = pitch_diameter - 2.5 * module
    root_radius = dedendum_diameter / 2
    
    # Circular pitch
    circular_pitch = math.pi * modified_module / 2
    
    # Profile shift related dimensions
    shift_distance = profile_shift * module
    shifted_pitch_diameter = pitch_diameter + 2 * profile_shift * module
    shifted_addendum_diameter = shifted_pitch_diameter + 2 * module
    shifted_dedendum_diameter = shifted_pitch_diameter - 2.5 * module
    shifted_pitch_radius = shifted_pitch_diameter / 2
    shifted_addendum_radius = shifted_addendum_diameter / 2
    shifted_dedendum_radius = shifted_dedendum_diameter / 2
    
    # Angular parameters
    alpha = ((math.sqrt((pitch_diameter ** 2) - (base_diameter ** 2))) / base_diameter) - modified_pressure_angle
    beta = math.pi / (2 * teeth)
    
    # Involute function
    def involute(angle):
        return math.tan(angle) - angle
    
    # Function to calculate the angle spanned by a tooth at a given diameter
    def tooth_angle_at_diameter(diameter):
        current_pressure_angle = math.acos(base_radius / (diameter / 2))
        angle = diameter * ((circular_pitch / pitch_diameter) + involute(modified_pressure_angle) - involute(current_pressure_angle))
        return angle / (diameter / 2)
    
    # Determine number of points for involute calculation
    if fast_compute:
        num_points = 50
    elif module < 15 and not fast_compute:
        num_points = int((89 * math.sqrt(module / 0.3)) - 3)
    else:  # module >= 15 and not fast_compute
        num_points = 425
    
    # Angles for tooth profile generation
    rotation_angle_2 = 2 * math.pi - alpha - beta
    rotation_angle = (math.pi / teeth) - alpha - beta
    
    # Create points for involute curves
    u_values = linear_space(0, math.sqrt(((addendum_diameter / base_diameter) ** 2) - 1), num_points)
    v_values = linear_space(0, math.sqrt(((addendum_diameter / base_diameter) ** 2) - 1), num_points)
    
    # Initialize coordinate arrays
    x_coords = []
    y_coords = []
    x2_coords = []
    y2_coords = []
    
    # Calculate involute curve coordinates
    for i in range(0, len(u_values)):
        x_coords.append(base_radius * (math.cos(u_values[i] + rotation_angle_2) + u_values[i] * math.sin(u_values[i] + rotation_angle_2)))
        y_coords.append(base_radius * (math.sin(u_values[i] + rotation_angle_2) - u_values[i] * math.cos(u_values[i] + rotation_angle_2)))
        x2_coords.append(base_radius * (math.cos(v_values[i] + rotation_angle_2) + v_values[i] * math.sin(v_values[i] + rotation_angle_2)))
        y2_coords.append(-base_radius * (math.sin(v_values[i] + rotation_angle_2) - v_values[i] * math.cos(v_values[i] + rotation_angle_2)))
    
    # Initialize arrays for profile shifted coordinates
    shifted_x = []
    shifted_y = []
    shifted_x2 = []
    shifted_y2 = []
    
    # Function to calculate profile shift angle
    def profile_shift_angle(radius, base_radius, pressure_angle, shift):
        local_pressure_angle = math.acos(base_radius / radius)
        diameter = 2 * radius
        angle = diameter * ((math.pi / (2 * teeth)) + (2 * shift * math.tan(pressure_angle) / teeth) + 
                           involute(pressure_angle) - involute(local_pressure_angle))
        return angle / radius
    
    # Apply profile shift if needed
    if profile_shift != 0:
        shift_rotation_angle = 2 * math.pi - profile_shift_angle(base_radius, base_radius, pressure_angle, profile_shift) / 2
        shifted_u_values = linear_space(0, math.sqrt(((shifted_addendum_diameter / base_diameter) ** 2) - 1), num_points)
        shifted_v_values = linear_space(0, math.sqrt(((shifted_addendum_diameter / base_diameter) ** 2) - 1), num_points)
        
        for i in range(0, len(shifted_u_values)):
            shifted_x.append(base_radius * (math.cos(shifted_u_values[i] + shift_rotation_angle) + 
                           shifted_u_values[i] * math.sin(shifted_u_values[i] + shift_rotation_angle)))
            shifted_y.append(base_radius * (math.sin(shifted_u_values[i] + shift_rotation_angle) - 
                           shifted_u_values[i] * math.cos(shifted_u_values[i] + shift_rotation_angle)))
            shifted_x2.append(base_radius * (math.cos(shifted_v_values[i] + shift_rotation_angle) + 
                            shifted_v_values[i] * math.sin(shifted_v_values[i] + shift_rotation_angle)))
            shifted_y2.append(-base_radius * (math.sin(shifted_v_values[i] + shift_rotation_angle) - 
                            shifted_v_values[i] * math.cos(shifted_v_values[i] + shift_rotation_angle)))
        
        x_coords = shifted_x
        y_coords = shifted_y
        x2_coords = shifted_x2
        y2_coords = shifted_y2
    
    # Determine helix direction
    helix_direction = 1
    if is_ring_gear:
        helix_direction = -1
    
    # Initialize helix related coordinates
    z_coords = []
    helix_x = []
    helix_y = []
    helix_x2 = []
    helix_y2 = []
    
    # Calculate helix parameters if helix angle is not zero
    try:
        helix_pitch = math.pi * pitch_diameter * math.cos(helix_angle) / math.sin(helix_angle)
        helix_angle_param = 10 * gear_width * 2 * math.pi / helix_pitch
        t2_values = linear_space(0, helix_angle_param, num_points)
        bb = helix_pitch / (2 * math.pi)
        
        for i in range(0, len(t2_values)):
            z_coords.append(bb * t2_values[i])
            helix_x.append(pitch_radius * math.cos(t2_values[i] + math.pi) + pitch_diameter)
            helix_y.append(pitch_radius * math.sin(-helix_direction * t2_values[i]))
            helix_x2.append(pitch_radius * math.cos(t2_values[i]))
            helix_y2.append(pitch_radius * math.sin(helix_direction * t2_values[i]))
    except:
        # Default values if helix angle is zero or calculation fails
        helix_pitch = 0
        helix_angle_param = 0
        t2_values = linear_space(0, 0, num_points)
        z_coords = t2_values
        helix_x = t2_values
        helix_y = t2_values
        helix_x2 = t2_values
        helix_y2 = t2_values
    
    # Calculate rotated coordinates for helical gears
    x_rot = []
    y_rot = []
    x2_rot = []
    y2_rot = []
    
    for i in range(0, len(u_values)):
        x_rot.append(base_radius * (math.cos(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param) + 
                   u_values[i] * math.sin(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param)) + 
                   (pitch_diameter - 2 * (pitch_radius * math.cos(helix_angle_param))))
        y_rot.append(base_radius * (math.sin(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param) - 
                   u_values[i] * math.cos(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param)) + 
                   2 * pitch_radius * math.sin(-helix_direction * helix_angle_param))
        x2_rot.append(base_radius * (math.cos(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param) + 
                    v_values[i] * math.sin(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param)) + 
                    (pitch_diameter - 2 * (pitch_radius * math.cos(helix_angle_param))))
        y2_rot.append(-base_radius * (math.sin(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param) - 
                     v_values[i] * math.cos(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param)) + 
                     2 * pitch_radius * math.sin(-helix_direction * helix_angle_param))
    
    # Calculate origin coordinates for helix
    try:
        chord_length = math.sqrt(2 * (pitch_diameter ** 2) * (1 - math.cos(-helix_direction * helix_angle_param)))
        angle_u = math.asin(pitch_diameter * math.sin(-helix_direction * helix_angle_param) / chord_length)
        x_new_origin = chord_length * math.cos(angle_u)
        y_new_origin = pitch_diameter * math.sin(-helix_direction * helix_angle_param)
    except:
        x_new_origin = 0
        y_new_origin = 0
    
    # Calculate additional rotated coordinates
    x_rot2 = []
    y_rot2 = []
    x3_rot = []
    y3_rot = []
    
    for i in range(0, len(u_values)):
        x_rot2.append(base_radius * (math.cos(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param) + 
                    u_values[i] * math.sin(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param)))
        y_rot2.append(base_radius * (math.sin(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param) - 
                    u_values[i] * math.cos(u_values[i] + rotation_angle_2 + helix_direction * helix_angle_param)))
        x3_rot.append(base_radius * (math.cos(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param) + 
                    v_values[i] * math.sin(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param)))
        y3_rot.append(-base_radius * (math.sin(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param) - 
                     v_values[i] * math.cos(v_values[i] + rotation_angle_2 - helix_direction * helix_angle_param)))
    
    # Return all calculated parameters and coordinates
    return (root_radius, x_coords, y_coords, x2_coords, y2_coords, num_points, tooth_angle_at_diameter(addendum_diameter), 
            addendum_radius, rotation_angle_2, alpha, beta, x_new_origin, y_new_origin, z_coords, helix_x, helix_y, 
            x_rot, y_rot, x2_rot, y2_rot, helix_x2, helix_y2, x_rot2, y_rot2, x3_rot, y3_rot, helix_angle_param, 
            base_radius, shifted_addendum_radius, shifted_dedendum_radius)

def sketch_tooth_profile(module, pressure_angle, root_radius, addendum_radius, x_coords, y_coords, x2_coords, y2_coords, 
                        num_points, tooth_tip_angle, is_ring_gear, is_standard, radial_thickness, new_comp):
    """
    Create a sketch of the tooth profile in Fusion 360.
    
    Args:
        module: Module of the gear
        pressure_angle: Pressure angle in radians
        root_radius: Dedendum/root radius
        addendum_radius: Addendum/tip radius
        x_coords, y_coords: Coordinates of first involute curve
        x2_coords, y2_coords: Coordinates of second involute curve
        num_points: Number of points for involute curve
        tooth_tip_angle: Angle at the tooth tip
        is_ring_gear: Boolean, True if it's a ring gear
        is_standard: Boolean, True if it's a standard gear
        radial_thickness: Radial thickness for ring gears
        new_comp: Component to add the sketch to
    
    Returns:
        Tuple containing (outer_profile, tooth_profile, sketch)
    """
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    rootComp = new_comp
    
    # Crown radius (for ring gears)
    outer_radius = addendum_radius + radial_thickness
    
    # Create sketches
    sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)
    sketch2 = rootComp.sketches.add(rootComp.xYConstructionPlane)
    
    # Define points for sketch
    origin = adsk.core.Point3D.create(0, 0, 0)
    origin2 = adsk.core.Point3D.create((root_radius + addendum_radius) / 10, 0, 0)
    
    # Get sketch curves
    circles = sketch2.sketchCurves.sketchCircles
    circles1 = sketch.sketchCurves.sketchCircles
    
    # Calculate pitch diameter and base diameter
    pitch_diameter = 2 * addendum_radius - 2 * module
    base_diameter = pitch_diameter * math.cos(pressure_angle)
    
    # Adjustment for potential base diameter issues
    v_adjustment = 0
    # if base_diameter <= 2 * root_radius:
    #     v_adjustment = 2 * root_radius - base_diameter
    
    # Create circles based on gear type
    if is_ring_gear and is_standard:
        # Standard ring gear
        outer_circle = circles.addByCenterRadius(origin, outer_radius / 10)
        inner_circle = circles.addByCenterRadius(origin, root_radius / 10)
    elif is_ring_gear and not is_standard:
        # Non-standard ring gear
        outer_circle = circles.addByCenterRadius(origin2, (addendum_radius + v_adjustment) / 10)
        inner_circle = circles.addByCenterRadius(origin2, outer_radius / 10)
    else:
        # Standard gear
        root_circle = circles.addByCenterRadius(origin, root_radius / 10)
    
    # Get outer profile based on gear type
    if is_standard:
        outer_profile = sketch2.profiles.item(0)
    else:
        outer_profile = sketch2.profiles.item(1)
    
    # Create collections for point management
    points = adsk.core.ObjectCollection.create()
    points2 = adsk.core.ObjectCollection.create()
    first_points = adsk.core.ObjectCollection.create()
    first_points2 = adsk.core.ObjectCollection.create()
    
    # Add first two points of each involute for special handling
    first_points.add(adsk.core.Point3D.create(x_coords[0] / 10, y_coords[0] / 10, 0))
    first_points.add(adsk.core.Point3D.create(x_coords[1] / 10, y_coords[1] / 10, 0))
    first_points2.add(adsk.core.Point3D.create(x2_coords[0] / 10, y2_coords[0] / 10, 0))
    first_points2.add(adsk.core.Point3D.create(x2_coords[1] / 10, y2_coords[1] / 10, 0))
    
    # Add remaining points to collections
    for i in range(1, num_points):
        points.add(adsk.core.Point3D.create(x_coords[i] / 10, y_coords[i] / 10, 0))
        points2.add(adsk.core.Point3D.create(x2_coords[i] / 10, y2_coords[i] / 10, 0))
    
    # Create splines for the involute curves
    spline1 = sketch.sketchCurves.sketchFittedSplines.add(points)
    spline2 = sketch.sketchCurves.sketchFittedSplines.add(points2)
    spline3 = sketch.sketchCurves.sketchFittedSplines.add(first_points)
    spline4 = sketch.sketchCurves.sketchFittedSplines.add(first_points2)
    
    # Create lines from origin to base of involutes
    lines = sketch.sketchCurves.sketchLines
    line1 = lines.addByTwoPoints(origin, first_points[0])
    line2 = lines.addByTwoPoints(origin, first_points2[0])
    
    # Create arc at tip of tooth
    tooth_tip = adsk.core.Point3D.create(x_coords[num_points - 1] / 10, y_coords[num_points - 1] / 10, 0)
    sketch.sketchCurves.sketchArcs.addByCenterStartSweep(origin, tooth_tip, tooth_tip_angle)
    
    # Create additional circle for non-standard gears to help with profile recognition
    if not is_standard:
        circles.addByCenterRadius(origin2, (addendum_radius + v_adjustment) / 10)
    
    # Get tooth profile
    if is_standard:
        tooth_profile = sketch.profiles.item(0)
    else:
        tooth_profile = sketch.profiles.item(0)
    
    return outer_profile, tooth_profile, sketch

def sketch_tooth_profile_shift(x, y, x2, y2, rva, rvf, aok, rb, m, z, ap, ah, X, newComp, normal_system=False):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        rootComp = newComp
        sketch = rootComp.sketches.add(rootComp.xYConstructionPlane)

        orig = adsk.core.Point3D.create(0, 0, 0)
        planes = rootComp.constructionPlanes
        extrudes = rootComp.features.extrudeFeatures
        sketch2 = rootComp.sketches.add(rootComp.xYConstructionPlane)

        sweeps = rootComp.features.sweepFeatures
        circles = sketch2.sketchCurves.sketchCircles
        circles1 = sketch.sketchCurves.sketchCircles

        modt = m / math.cos(ah) if normal_system else m
        apt = math.atan(math.tan(ap) / math.cos(ah)) if normal_system else ap
        dp = modt * z
        dvp = dp + 2 * X * m
        rvf = (dvp - 2.5 * m) / 2
        db = dp * math.cos(apt)
        alpa = math.sqrt((dp * dp) - (db * db)) / db - apt
        alpa2 = math.pi / z - (alpa + math.pi / (2 * z))
        df = dp - 2.5 * m
        rf = df / 2
        adesc = math.atan((rb * math.sin(alpa2)) / (rb * math.cos(alpa2) + X * m))

        coordenx = rvf * math.cos(adesc)
        coordeny = rvf * math.sin(adesc)
        lux = adsk.core.Point3D.create(coordenx / 10, coordeny / 10, 0)
        rascahuele = adsk.core.Point3D.create(coordenx / 10, -coordeny / 10, 0)
        orig = adsk.core.Point3D.create(0, 0, 0)
        points = adsk.core.ObjectCollection.create()
        points2 = adsk.core.ObjectCollection.create()
        puntos = adsk.core.ObjectCollection.create()
        puntos2 = adsk.core.ObjectCollection.create()
        puntos.add(adsk.core.Point3D.create(x[0] / 10, y[0] / 10, 0))
        puntos.add(adsk.core.Point3D.create(x[1] / 10, y[1] / 10, 0))
        puntos2.add(adsk.core.Point3D.create(x2[0] / 10, y2[0] / 10, 0))
        puntos2.add(adsk.core.Point3D.create(x2[1] / 10, y2[1] / 10, 0))
        for i in range(1, aok):
            points.add(adsk.core.Point3D.create(x[i] / 10, y[i] / 10, 0))
            points2.add(adsk.core.Point3D.create(x2[i] / 10, y2[i] / 10, 0))
        spa = sketch.sketchCurves.sketchFittedSplines.add(points)
        spb = sketch.sketchCurves.sketchFittedSplines.add(points2)
        spc = sketch.sketchCurves.sketchFittedSplines.add(puntos)
        spd = sketch.sketchCurves.sketchFittedSplines.add(puntos2)
        lines = sketch.sketchCurves.sketchLines
        line1 = lines.addByTwoPoints(orig, puntos[0])
        line2 = lines.addByTwoPoints(orig, puntos2[0])
        def sigmaPS(rt, rb, ap, X):
            alphat = math.acos(rb / rt)
            dt = 2 * rt
            Tt = dt * ((math.pi / (2 * z)) + (2 * X * math.tan(ap) / (z)) + (math.tan(ap) - ap) - (math.tan(alphat) - alphat))
            sigm = Tt / rt
            return sigm
        pointo = adsk.core.Point3D.create(x[aok - 1] / 10, y[aok - 1] / 10, 0)
        sketch.sketchCurves.sketchArcs.addByCenterStartSweep(orig, pointo, sigmaPS(rva, rb, ap, X))
        sketcht = rootComp.sketches.item(0)
        vec = adsk.core.Vector3D.create(0, 1, 0)
        vec.add(adsk.core.Vector3D.create(0, 10, 0))
        objc = adsk.core.ObjectCollection.create()
        prof = sketch.profiles.item(0)
        circles.addByCenterRadius(orig, rvf / 10)
        prof2 = sketch2.profiles.item(0)

        return prof2, prof, sketch
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def extrude_profile(profile, gear_width, new_component, u='Escribe: NewBody,Join o Cut'):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        if u == 'NewBody':
            operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        elif u == 'Join':
            operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        elif u == 'Cut':
            operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        extrudes = root_comp.features.extrudeFeatures
        extInput = extrudes.createInput(profile, operation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(gear_width))
        extrusion = extrudes.add(extInput)
        try:
            face = extrusion.endFaces.item(0)
            return extrusion, face
        except:
            return extrusion
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def create_circular_pattern(axis_line, is_conical, outer_radius, root_radius, teeth, tooth_body, is_standard, gear_width, new_component, u='Escribe si la operación fue Cut o Join para el perfil del diente'):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = new_component
    if not is_standard and not is_conical:
        sketch = root_comp.sketches.add(root_comp.xYConstructionPlane)
        lines = sketch.sketchCurves.sketchLines
        z_line = lines.addByTwoPoints(adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, 0), adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, gear_width))
        axis_line = z_line
        sketch.isVisible = False
    elif is_conical and is_standard:
        axis_line = axis_line
    elif not is_conical and is_standard:
        zAxis = root_comp.zConstructionAxis
        axis_line = zAxis
    input_entities = adsk.core.ObjectCollection.create()
    input_entities.add(tooth_body)
    circular_feats = root_comp.features.circularPatternFeatures
    circular_input = circular_feats.createInput(input_entities, axis_line)
    circular_input.quantity = adsk.core.ValueInput.createByReal(teeth)
    circular_input.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    circular_input.isSymmetric = False
    compute_option = 0
    if u == 'Cut':
        compute_option = 1
    circular_input.patternComputeOption = compute_option
    circular_feats.add(circular_input)

def sweep_helix_profile(is_profile_shift, outer_radius, root_radius, has_ring, helix_angle, sketch, num_points, x_list, y_list, z_list, gear_width, clockwise, new_component, u='Escribe: NewBody,Join o Cut'):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        sweeps = root_comp.features.sweepFeatures
        lines = sketch.sketchCurves.sketchLines
        pvec = adsk.core.Point3D.create(0, 0, gear_width / 1.25)
        origin = adsk.core.Point3D.create(0, 0, 0)
        delta_m = (outer_radius - root_radius) / 2.25
        pitch_radius = outer_radius - delta_m
        pah = math.pi * (2 * pitch_radius) * (math.cos(helix_angle) / math.sin(helix_angle))
        helix_turn = (10 * (gear_width / 1.25) * 2 * math.pi) / pah
        if clockwise:
            helix_turn = -helix_turn
        sweep_index = 0
        if not has_ring:
            prof = sketch.profiles.item(sweep_index)
        else:
            prof = sketch.profiles.item(sweep_index)
            origin = adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, 0)
            pvec = adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, gear_width / 1.25)
        if u == 'NewBody':
            operation = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        elif u == 'Join':
            operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
        elif u == 'Cut':
            operation = adsk.fusion.FeatureOperations.CutFeatureOperation
        lpath = lines.addByTwoPoints(origin, pvec)
        path2 = new_component.features.createPath(lpath, False)
        esq = root_comp.sketches.item(root_comp.sketches.count - 2)
        prof = esq.profiles.item(0)
        sweepInput = sweeps.createInput(prof, path2, operation)
        sweepInput.twistAngle = adsk.core.ValueInput.createByReal(helix_turn)
        sweep = sweeps.add(sweepInput)
        return sweep, u
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def cut_excess_profile(outer_radius, root_radius, radial_thickness, gear_width, new_component):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        rcor = outer_radius + radial_thickness
        sketch = root_comp.sketches.add(root_comp.xYConstructionPlane)
        origin = adsk.core.Point3D.create(0, 0, 0)
        origin2 = adsk.core.Point3D.create((root_radius + outer_radius) / 10, 0, 0)
        extrudes = root_comp.features.extrudeFeatures
        circles1 = sketch.sketchCurves.sketchCircles
        circles1.addByCenterRadius(origin2, (outer_radius + root_radius + rcor) / 10)
        circles1.addByCenterRadius(origin2, (rcor) / 10)
        prof = sketch.profiles.item(0)
        extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2 * gear_width + 1))
        extrusion = extrudes.add(extInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def mirror_body(sketch, gear_width, face, new_component):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        sweeps = root_comp.features.sweepFeatures
        lines = sketch.sketchCurves.sketchLines
        pvec = adsk.core.Point3D.create(0, 0, gear_width)
        origin = adsk.core.Point3D.create(0, 0, 0)
        origin3 = adsk.core.Point3D.create(0, 0, gear_width)
        sketchLineOne = lines.addByTwoPoints(origin, origin3)
        distance_val = adsk.core.ValueInput.createByReal(gear_width)
        planes = root_comp.constructionPlanes
        planeInput = planes.createInput()
        planeInput.setByOffset(root_comp.xYConstructionPlane, distance_val)
        if not face:
            uj = planes.add(planeInput)
        else:
            uj = face
        count_bodies = root_comp.bRepBodies.count
        TargetBody = root_comp.bRepBodies.item(count_bodies - 1)
        bodyco = adsk.core.ObjectCollection.create()
        bodyco.add(TargetBody)
        mirrorFeatures = root_comp.features.mirrorFeatures
        mirrorInput = mirrorFeatures.createInput(bodyco, uj)
        mirrorInput.patternComputeOption = 0
        mirrorFeatures.add(mirrorInput)
        Toolbodies = adsk.core.ObjectCollection.create()
        count_bodies2 = root_comp.bRepBodies.count
        Toolbodies.add(root_comp.bRepBodies.item(count_bodies2 - 1))
        CombineInput = root_comp.features.combineFeatures.createInput(TargetBody, Toolbodies)
        CombineInput.operation = 0
        CombineInput.isKeepToolBodies = False
        CombineInput.isNewComponent = False
        root_comp.features.combineFeatures.add(CombineInput)
        sketch.isVisible = False
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def plane_cut(radial_thickness, outer_radius, root_radius, gear_width, is_standard):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = design.rootComponent
        planes = root_comp.constructionPlanes
        extrudes = root_comp.features.extrudeFeatures
        planeInput = planes.createInput()
        origin2 = adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, 0)
        origin = adsk.core.Point3D.create(0, 0, 0)
        origin_ref = origin
        if is_standard:
            origin_ref = origin2
        distance_val = adsk.core.ValueInput.createByReal(1.25 * gear_width + 1)
        planeInput.setByOffset(root_comp.xYConstructionPlane, distance_val)
        uj = planes.add(planeInput)
        sketch = root_comp.sketches.add(uj)
        circles = sketch.sketchCurves.sketchCircles
        circles.addByCenterRadius(origin_ref, (outer_radius + radial_thickness) / 10)
        extInput = extrudes.createInput(sketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-0.25 * gear_width - 1))
        extrudes.add(extInput)
        count = root_comp.bRepBodies.count
        tooth = root_comp.bRepBodies.item(count - 1)
        uj.isVisible = False
        return tooth
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def combine_bodies(teeth, new_component):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        count_target = (root_comp.bRepBodies.count) - 1
        count_tool = root_comp.bRepBodies.count
        TargetBody = root_comp.bRepBodies.item(count_target - teeth)
        Toolbodies = adsk.core.ObjectCollection.create()
        for i in range(count_tool - teeth, count_tool):
            Toolbodies.add(root_comp.bRepBodies.item(i))
        CombineInput = root_comp.features.combineFeatures.createInput(TargetBody, Toolbodies)
        CombineInput.operation = 0
        CombineInput.isKeepToolBodies = False
        CombineInput.isNewComponent = False
        root_comp.features.combineFeatures.add(CombineInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def cut_crown(gear_width, pitch_radius, root_radius, outer_radius, radial_thickness, new_component):
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = app.activeProduct
        root_comp = new_component
        extrudes = root_comp.features.extrudeFeatures
        origin = adsk.core.Point3D.create(0, 0, 0)
        origin2 = adsk.core.Point3D.create((outer_radius + root_radius) / 10, 0, 0)
        sketchc = root_comp.sketches.add(root_comp.xYConstructionPlane)
        circles = sketchc.sketchCurves.sketchCircles
        rcor = (outer_radius + radial_thickness) / 10
        circles.addByCenterRadius(origin2, (rcor) / 1)
        circles.addByCenterRadius(origin2, (rcor + (2 * root_radius - 2 * pitch_radius) / 10) / 1)
        prof = sketchc.profiles.item(1)
        extInput = extrudes.createInput(prof, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extInput.setDistanceExtent(False, adsk.core.ValueInput.createByReal(2 * gear_width))
        extrudes.add(extInput)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def move_occurrence(x, y, z, occurrence):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = design.rootComponent
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(x, y, z)
    occurrence.transform2 = transform


def create_planetary_gear(module, teeth, pressure_angle, fast_compute, gear_width, new_component):
    list3 = calculate_gear_parameters(module, teeth, pressure_angle, 0, gear_width, False, 0, fast_compute)
    root_radius = list3[0]
    involute_x = list3[1]
    involute_y = list3[2]
    involute_x2 = list3[3]
    involute_y2 = list3[4]
    num_points = list3[5]
    tooth_tip_angle = list3[6]
    outer_radius = list3[7]
    prof = sketch_tooth_profile(module, pressure_angle, root_radius, outer_radius, involute_x, involute_y, involute_x2, involute_y2, num_points, tooth_tip_angle, False, True, 0, new_component)
    extrude_profile(prof[0], gear_width, new_component, 'NewBody')
    diente = extrude_profile(prof[1], gear_width, new_component, 'NewBody')
    create_circular_pattern(1, False, outer_radius, root_radius, teeth, diente[0], True, gear_width, new_component, 'Join')
    combine_bodies(teeth, new_component)
def hide_bodies(new_component):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = new_component
    root_comp = design.rootComponent
    bodies_list = []
    for j in range(0, root_comp.occurrences.count):
        Comp = root_comp.occurrences.item(j)
        count = Comp.bRepBodies.count
        for i in range(0, count):
            body = Comp.bRepBodies.item(i)
            bodies_list.append(body)
            body.isVisible = False
    return bodies_list

def show_hidden_bodies(bodies, new_component):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = new_component
    for i in range(0, len(bodies)):
        bodies[i].isVisible = True

def create_new_tab():
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = design.rootComponent
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)

def create_text(text, outer_radius):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = design.rootComponent
    sketch = root_comp.sketches.add(root_comp.xYConstructionPlane)
    sketchTexts = sketch.sketchTexts
    origin = adsk.core.Point3D.create(-8.5, (outer_radius) / 10, 0)
    sketchTextInput = sketchTexts.createInput(text, 1.0, origin)
    sketchTextInput.textStyle = adsk.fusion.TextStyles.TextStyleBold
    sketchText = sketchTexts.add(sketchTextInput)

def hide_planes():
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = design.rootComponent
    planes = root_comp.constructionPlanes
    count = planes.count
    for i in range(0, count):
        planes.item(i).isVisible = False


def create_technical_specification(pressure_angle_fc, has_ring, is_helical, is_conical, is_worm, is_profile_shift, module, pressure_angle, teeth, helix_angle, radial_thickness, teeth_secondary, worm_radius, profile_shift, new_component, normal_system=False):
    app = adsk.core.Application.get()
    ui = app.userInterface
    design = app.activeProduct
    root_comp = new_component
    cadena = "FC=" + str(pressure_angle_fc) + " " + str(module) + " PA=" + str(radians_to_degrees(pressure_angle)) + "°" + " z=" + str(teeth)
    if has_ring and not is_helical:
        cadena = cadena + "\n" + "Radial Thickness=" + str(radial_thickness)
    if is_helical and not has_ring:
        helicalSystem = "Radial System, "
        if normal_system:
            helicalSystem = "Normal System, "
        cadena = helicalSystem + cadena + " HA=" + str(round(radians_to_degrees(helix_angle), 1)) + "°"
    if is_helical and has_ring:
        helicalSystem = "Radial System, "
        if normal_system:
            helicalSystem = "Normal System, "
        cadena = helicalSystem + cadena + " HA=" + str(round(radians_to_degrees(helix_angle), 1)) + "°" + "\n" + "Radial Thickness=" + str(radial_thickness)
    if is_conical:
        cadena = cadena + "\n" + "z2=" + str(teeth_secondary)
    if is_worm:
        cadena = cadena + "\n" + "Screw Radius=" + str(worm_radius)
    if is_profile_shift and not is_helical:
        cadena = cadena + "\n" + "X=" + str(profile_shift)
    if is_profile_shift and is_helical:
        helicalSystem = "Radial System, "
        if normal_system:
            helicalSystem = "Normal System, "
        cadena = helicalSystem + cadena + " HA=" + str(round(radians_to_degrees(helix_angle), 1)) + "°" + "\n" + "X=" + str(profile_shift)
    sketch = root_comp.sketches.add(root_comp.xYConstructionPlane)
    sketchTexts = sketch.sketchTexts
    oint = adsk.core.Point3D.create(0, 0, 0)
    sketchTextInput = sketchTexts.createInput(cadena, .30, oint)
    sketchTextInput.textStyle = adsk.fusion.TextStyles.TextStyleBold
    sketchText = sketchTexts.add(sketchTextInput)
    sketch.isVisible = False

command_cache = {}
def get(obj, key, default=None):
    if type(obj) in command_cache and key in command_cache[type(obj)]:
        return command_cache[type(obj)][key]
    return default 
def set(obj_type, key, value):
    global command_cache
    if obj_type not in command_cache:
        command_cache[obj_type] = {}
    command_cache[obj_type][key] = value
def save_params(command_type, command_inputs):
    for i in range(int(command_inputs.count)):
        try:
            input = command_inputs.item(i)
            if type(input) not in [adsk.core.ButtonRowCommandInput, adsk.core.DropDownCommandInput]:
                val = input.value
                if hasattr(input, 'unitType') and input.unitType == 'deg':
                    val = radians_to_degrees(val) 
                set(command_type, input.id, val)
        except Exception as e:
            print('Exception')
            print(e)


"""
The command handler functions below are taken from GF-Gear-Generator, which showcases the use of the Fusion 360 API calls.
Minimal changes were made to adapt them to the current context.
The original code is licensed under the BSD-3-Clause license.
"""
class cmdDefPressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        cmd.isExecutedWhenPreEmpted = False


        # Standard dropdown menu
        standard = inputs.addDropDownCommandInput('standard', 'Standard', adsk.core.DropDownStyles.TextListDropDownStyle)
        standard.listItems.add('Metric', True)
        standard.listItems.add('English', False)

        # Fusion's default units are cm, since your units are mm you'll need to divide that value with 10
        # a ValueInput = 1 will show as 10mm
        aaok=inputs.addBoolValueInput('FastCompute','Fast Compute',True,'', get(self, 'FastCompute', defaultfc))
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(get(self, 'Module', .03)))
        pitch = inputs.addValueInput('Pitch', 'Pitch [in]', 'in', adsk.core.ValueInput.createByReal(get(self, 'Pitch', 23.460456)))
        pitch.isVisible = False
        # u=inputs.addDropDownCommandInput('DropDownCommandInput1','Module [mm]', get(self, 'DropDownCommandInput1', 1))
        # qty=u.listItems
        # qty.add('0.3 mm',True,'si')
        # for nn in range(0,len(list)):
        #     qty.add(list[nn],False)
        inputs.addIntegerSpinnerCommandInput('Z', 'Number of teeth [ ]', 6, 250, 1, get(self, 'Z', 17))
        inputs.addValueInput('GearHeight_mm', 'Gear height [mm]', 'mm', adsk.core.ValueInput.createByReal(get(self, 'GearHeight_mm', 1)))
        inHeight = inputs.addValueInput('GearHeight_in', 'Gear height [in]', 'in', adsk.core.ValueInput.createByReal(get(self, 'GearHeight_in', 1.27)))
        inHeight.isVisible = False
        inputs.addFloatSpinnerCommandInput('PressureAngle', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, get(self, 'PressureAngle', 14.5))
        
        # When any input changes, the following handler triggers
        onInputChanged = ExternalGear_ChangedHandler()
        cmd.inputChanged.add(onInputChanged)
        handlers.append(onInputChanged) 

        # con esto vinculo al boton OK
        onExecute=cmdDefOKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)

# Event handler for the inputChanged event.
class ExternalGear_ChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        try:
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            
            # Gets the command input that was changed and its parent's command inputs
            changedInput = eventArgs.input
            inputs2=changedInput.parentCommand.commandInputs
            
            # In the case that's the standard, it switches visibiity of module/pitch value inputs as well as gear height
            if changedInput.id == 'standard':
                if changedInput.selectedItem.name == 'English':
                    # English system is selected

                    # Tooth Size Value Inputs
                    inputs2.itemById('Module').isVisible = False
                    inputs2.itemById('Pitch').isVisible = True
                    
                    # Height Value Inputs
                    inputs2.itemById('GearHeight_mm').isVisible = False
                    inputs2.itemById('GearHeight_in').isVisible = True

                elif changedInput.selectedItem.name == 'Metric':
                    # Metric system is selected

                    # Tooth Size Value Inputs
                    inputs2.itemById('Module').isVisible = True
                    inputs2.itemById('Pitch').isVisible = False
                    
                    # Height Value Inputs
                    inputs2.itemById('GearHeight_mm').isVisible = True
                    inputs2.itemById('GearHeight_in').isVisible = False
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
 

class cmdDef9PressedEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        app=adsk.core.Application.get()
        ui=app.userInterface
        cmd=args.command
        inputs=cmd.commandInputs
        cmd.isExecutedWhenPreEmpted = False

        # Standard dropdown menu
        standard = inputs.addDropDownCommandInput('standard', 'Standard', adsk.core.DropDownStyles.TextListDropDownStyle)
        standard.listItems.add('Metric', True)
        standard.listItems.add('English', False)

        inputs.addBoolValueInput('FastCompute','Fast Compute', True, '', get(self, 'FastCompute', defaultfc))
        inputs.addFloatSpinnerCommandInput('X','Profile shifting coef "X" [ ]','',-3,3,.01, get(self, 'X', 0))
        inputs.addValueInput('Module', 'Module [mm]', 'mm', adsk.core.ValueInput.createByReal(get(self, 'Module', .03)))
        pitch = inputs.addValueInput('Pitch', 'Pitch [in]', 'in', adsk.core.ValueInput.createByReal(get(self, 'Pitch', 23.460456)))
        pitch.isVisible = False

        inputs.addIntegerSpinnerCommandInput('Z', 'Number of teeth [ ]', 6, 250, 1, get(self, 'Z', 17))
        inputs.addValueInput('GearHeight_mm', 'Gear height [mm]', 'mm', adsk.core.ValueInput.createByReal(get(self, 'GearHeight_mm', 1)))
        inHeight = inputs.addValueInput('GearHeight_in', 'Gear height [in]', 'in', adsk.core.ValueInput.createByReal(get(self, 'GearHeight_in', 1.27)))
        inHeight.isVisible = False

        inputs.addFloatSpinnerCommandInput('PressureAngle', 'Pressure angle [°]', 'deg', 14.5, 30, 0.5, get(self, 'PressureAngle', 14.5))
        
        # When any input changes, the following handler triggers
        onInputChanged = ExternalGear_ChangedHandler()
        cmd.inputChanged.add(onInputChanged)
        handlers.append(onInputChanged)
        
        # con esto vinculo al boton OK
        onExecute=cmdDef9OKButtonPressedEventHandler()
        cmd.execute.add(onExecute)
        handlers.append(onExecute)


class cmdDefOKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        try:
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            design = app.activeProduct
            inputs2=eventArgs.command.commandInputs
            aaok=inputs2.itemById('FastCompute').value
            z=inputs2.itemById('Z').value

            standard = inputs2.itemById('standard').selectedItem.name
            # Fusion's default units are cm, since you're using mm you'll have to multiply the value per 20
            if standard == 'Metric':
                m=inputs2.itemById('Module').value*10
                textmodule = "m= "+ inputs2.itemById('Module').expression
                anchoeng=inputs2.itemById('GearHeight_mm').value
            elif standard == 'English':
                m=25.4/(inputs2.itemById('Pitch').value/2.54)
                textmodule = "p= "+ inputs2.itemById('Pitch').expression
                anchoeng=inputs2.itemById('GearHeight_in').value

            ap=inputs2.itemById('PressureAngle').value

            save_params(cmdDefPressedEventHandler, inputs2)
            nuOfOps = design.timeline.count
            design = adsk.fusion.Design.cast(app.activeProduct)
            root = design.activeComponent
            newComp = root.occurrences.addNewComponent(adsk.core.Matrix3D.create()).component
            hb=hide_bodies(newComp)
            create_planetary_gear(m,z,ap,aaok,anchoeng,newComp)
            show_hidden_bodies(hb, newComp)
            create_technical_specification(aaok,False,False,False,False,False, textmodule,ap,z,0,0,0,0,0,newComp)
            create_timeline_group(design.timeline.count - nuOfOps)
            # nummerop=5


        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
#Ejecución para crear un engrane DR

class cmdDef9OKButtonPressedEventHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self,args):
        try:
            eventArgs=adsk.core.CommandEventArgs.cast(args)
            app=adsk.core.Application.get()
            ui=app.userInterface
            design = adsk.fusion.Design.cast(app.activeProduct)
            root = design.activeComponent
            inputs2=eventArgs.command.commandInputs
            save_params(cmdDef9PressedEventHandler, eventArgs.command.commandInputs)
            nuOfOps = design.timeline.count
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            newComp = occ.component
            aaok=inputs2.itemById('FastCompute').value
            z=inputs2.itemById('Z').value
            
            standard = inputs2.itemById('standard').selectedItem.name
            if standard == 'Metric':
                m=inputs2.itemById('Module').value*10
                anchoeng=inputs2.itemById('GearHeight_mm').value

                # Text expressions
                textmodule = "m= "+ inputs2.itemById('Module').expression

            elif standard == 'English':
                m=25.4/(inputs2.itemById('Pitch').value/2.54)
                anchoeng=inputs2.itemById('GearHeight_in').value

                # Text expressions
                textmodule = "p= "+ inputs2.itemById('Pitch').expression

            ap=inputs2.itemById('PressureAngle').value
            X=inputs2.itemById('X').value
            if abs(X)==0:
                X=0
            hb=hide_bodies(newComp)
            list3=calculate_gear_parameters(m,z,ap,0,anchoeng,False,X,aaok)
            rf=list3[0]
            x=list3[1]
            y=list3[2]
            x2=list3[3]
            y2=list3[4]
            aok=list3[5]
            Ttda=list3[6]
            ra=list3[7]
            rva=list3[28]
            rvf=list3[29]
            rb=list3[27]
            if abs(X)==0:
                create_planetary_gear(m,z,ap,aaok,anchoeng, newComp)
                create_technical_specification(aaok,False,False,False,False,False,textmodule,ap,z,0,0,0,0,0, newComp)
            else:
                prof=sketch_tooth_profile_shift(x,y,x2,y2,rva,rvf,aok,rb,m,z,ap, 0, X, newComp)
                op='NewBody'
                extrude_profile(prof[0],anchoeng, newComp, 'NewBody')
                diente=extrude_profile(prof[1],anchoeng,newComp, op)
                create_circular_pattern(1,False,ra,rf,z,diente[0],True,anchoeng, newComp, 'Join')
                create_technical_specification(aaok,False,False,False,False,True,textmodule,ap,z,0,0,0,0,X, newComp)
                combine_bodies(z, newComp)
            create_timeline_group(design.timeline.count - nuOfOps)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
        show_hidden_bodies(hb, newComp)


tbPanel = None
def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        workSpace = ui.workspaces.itemById('FusionSolidEnvironment')
        tbPanels = workSpace.toolbarPanels

        global tbPanel
        tbPanel = tbPanels.itemById('NewPanel')
        if tbPanel:
            tbPanel.deleteMe()
        tbPanel = tbPanels.add('NewPanel', 'GEAR GENERATOR', 'SelectPanel', False)
        cmdDef =ui.commandDefinitions.itemById('NC1')
        cmdDef9=ui.commandDefinitions.itemById('NC9')
        if cmdDef:
            cmdDef.deleteMe()
        if cmdDef9:
            cmdDef9.deleteMe()
        cmdDef=ui.commandDefinitions.addButtonDefinition('NC1', 'Spur Gear', 'Creates a standard spur gear.','Resources/Recto')
        cmdDef9=ui.commandDefinitions.addButtonDefinition('NC9','Profile Shifted Spur Gear','Creates a spur gear using "Profile Shifting".','Resources/Recto')
        cmdDefcontrol=tbPanel.controls.addCommand(cmdDef)
        tbPanel.controls.addSeparator()
        tbPanel.controls.addCommand(cmdDef9)
        cmdDefcontrol.isPromotedByDefault=True
        cmdDefcontrol.isPromoted=True
        cmdDefPressed=cmdDefPressedEventHandler()
        cmdDef.commandCreated.add(cmdDefPressed)
        handlers.append(cmdDefPressed)

        cmdDef9Pressed=cmdDef9PressedEventHandler()
        cmdDef9.commandCreated.add(cmdDef9Pressed)
        handlers.append(cmdDef9Pressed)
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if tbPanel:
            tbPanel.deleteMe()
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))