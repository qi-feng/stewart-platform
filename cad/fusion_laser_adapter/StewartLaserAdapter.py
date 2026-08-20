import adsk.core
import adsk.fusion
import traceback


def axis_data(box):
    spans = [
        box.maxPoint.x - box.minPoint.x,
        box.maxPoint.y - box.minPoint.y,
        box.maxPoint.z - box.minPoint.z,
    ]
    axis = spans.index(min(spans))
    center = adsk.core.Point3D.create(
        (box.minPoint.x + box.maxPoint.x) / 2,
        (box.minPoint.y + box.maxPoint.y) / 2,
        (box.minPoint.z + box.maxPoint.z) / 2,
    )
    vector = [
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0),
        adsk.core.Vector3D.create(0, 0, 1),
    ][axis]
    return axis, spans[axis], center, vector


def shifted(point, vector, distance):
    result = point.copy()
    offset = vector.copy()
    offset.scaleBy(distance)
    result.translateBy(offset)
    return result


def add_cylinder(component, start, end, radius, name):
    manager = adsk.fusion.TemporaryBRepManager.get()
    temporary = manager.createCylinderOrCone(start, radius, end, radius)
    body = component.bRepBodies.add(temporary)
    body.name = name
    return body


def combine(component, target, tool, operation):
    tools = adsk.core.ObjectCollection.create()
    tools.add(tool)
    feature_input = component.features.combineFeatures.createInput(target, tools)
    feature_input.operation = operation
    feature_input.isKeepToolBodies = False
    return component.features.combineFeatures.add(feature_input)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open the STEP assembly in the Design workspace first.')
            return

        selection = ui.selectEntity(
            'Select the solid body named stewart-platform_adapter',
            'SolidBodies',
        )
        source = adsk.fusion.BRepBody.cast(selection.entity)
        if not source:
            ui.messageBox('The selected object is not a solid body.')
            return

        root = design.rootComponent
        occurrence = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        component = occurrence.component
        component.name = 'Laser Adapter Modification'

        copied = component.features.copyPasteBodies.add(source)
        adapter = copied.bodies.item(0) if copied and copied.bodies.count else component.bRepBodies.item(0)
        adapter.name = 'Adapter - reinforced 10mm laser hole'

        _, thickness, center, normal = axis_data(adapter.boundingBox)

        # Fusion's internal length unit is cm.
        through_radius = 0.53       # 10.6 mm diameter
        relief_radius = 0.70        # 14.0 mm cable/strain-relief pocket
        ring_outer_radius = 1.50    # 30.0 mm outside diameter
        ring_height = 0.40          # 4.0 mm reinforcement
        clamp_inner_radius = 0.52   # 10.4 mm bore
        clamp_outer_radius = 1.30   # 26.0 mm outside diameter
        clamp_height = 2.00         # 20.0 mm tall

        start = shifted(center, normal, -(thickness / 2 + 0.5))
        end = shifted(center, normal, thickness / 2 + ring_height + 0.5)
        hole = add_cylinder(component, start, end, through_radius, '10.6 mm through-hole tool')
        combine(component, adapter, hole, adsk.fusion.FeatureOperations.CutFeatureOperation)

        # Add a 30 mm OD x 4 mm reinforcing boss on the positive-normal face.
        ring_start = shifted(center, normal, thickness / 2 - 0.02)
        ring_end = shifted(center, normal, thickness / 2 + ring_height)
        boss = add_cylinder(component, ring_start, ring_end, ring_outer_radius, 'Reinforcement boss')
        combine(component, adapter, boss, adsk.fusion.FeatureOperations.JoinFeatureOperation)
        boss_hole = add_cylinder(component, ring_start, ring_end, through_radius, 'Boss bore tool')
        combine(component, adapter, boss_hole, adsk.fusion.FeatureOperations.CutFeatureOperation)

        # Add a 14 mm diameter stepped cable pocket from the opposite face,
        # stopping 2 mm before the reinforced/top face.
        relief_start = shifted(center, normal, -(thickness / 2 + 0.2))
        relief_end = shifted(center, normal, thickness / 2 - 0.20)
        relief = add_cylinder(component, relief_start, relief_end, relief_radius, '14 mm cable relief tool')
        combine(component, adapter, relief, adsk.fusion.FeatureOperations.CutFeatureOperation)

        # Create a separate collar for inspection. It is intentionally not joined
        # to the adapter so it can be exported/reprinted independently.
        clamp_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        clamp_component = clamp_occ.component
        clamp_component.name = '10mm Laser Split Clamp'
        clamp_start = adsk.core.Point3D.create(0, 0, 0)
        clamp_end = adsk.core.Point3D.create(0, 0, clamp_height)
        clamp = add_cylinder(clamp_component, clamp_start, clamp_end, clamp_outer_radius, 'Laser clamp')
        clamp_bore = add_cylinder(clamp_component, clamp_start, clamp_end, clamp_inner_radius, '10.4 mm clamp bore tool')
        combine(clamp_component, clamp, clamp_bore, adsk.fusion.FeatureOperations.CutFeatureOperation)

        # Cut a full-height radial flex slot. This collar can be squeezed with a
        # small hose clamp or zip tie for the first fit test.
        sketches = clamp_component.sketches
        sketch = sketches.add(clamp_component.xYConstructionPlane)
        lines = sketch.sketchCurves.sketchLines
        lines.addTwoPointRectangle(
            adsk.core.Point3D.create(-0.15, 0, 0),
            adsk.core.Point3D.create(0.15, clamp_outer_radius + 0.2, 0),
        )
        profile = sketch.profiles.item(0)
        ext_input = clamp_component.features.extrudeFeatures.createInput(
            profile, adsk.fusion.FeatureOperations.CutFeatureOperation
        )
        ext_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal(clamp_height))
        clamp_component.features.extrudeFeatures.add(ext_input)

        ui.messageBox(
            'Created “Laser Adapter Modification” and “10mm Laser Split Clamp”.\n\n'
            'Inspect the hole, rib intersection, cable pocket, and clamp fit before exporting.'
        )
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    pass
