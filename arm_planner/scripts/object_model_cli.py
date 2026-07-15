#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA Object Model CLI
Manage the object knowledge base from the command line.

Usage:
  python3 object_model_cli.py list
  python3 object_model_cli.py show cup
  python3 object_model_cli.py register --name green_mug --mass 0.3
  python3 object_model_cli.py update cup --mass 0.3 --grip-force 0.4
  python3 object_model_cli.py stats
  python3 object_model_cli.py export --format yaml --output library.yaml
  python3 object_model_cli.py import objects_library.yaml
═══════════════════════════════════════════════════════════════
"""
import argparse
import json
import os
import sys
from datetime import datetime

# Add parent to path for imports
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..'))

from arm_planner.object_model_db import ObjectModelDB, ObjectModel


def cmd_list(db: ObjectModelDB, args):
    """List all known objects."""
    objects = db.list_objects()
    if not objects:
        print("No objects registered. Run 'import' to load defaults.")
        return

    print(f"\n{'Class':>20} {'Material':>12} {'Grasps':>7} "
          f"{'Success':>8} {'Force':>6} {'Approach':>10} Last Seen")
    print("─" * 90)

    for obj in objects:
        rate = f"{obj.grasp_success_rate:.0%}" if obj.times_grasped > 0 else "—"
        last = obj.last_seen[:10] if obj.last_seen else "never"
        print(f"{obj.class_name:>20} {obj.material:>12} "
              f"{obj.times_grasped:>7} {rate:>8} "
              f"{obj.grip_force_scale:>5.0%} {obj.approach_style:>10} {last}")

    print(f"\nTotal: {len(objects)} object(s)\n")


def cmd_show(db: ObjectModelDB, args):
    """Show full details for an object."""
    model = db.get_model(args.name)
    if model is None:
        print(f"Object '{args.name}' not found")
        return

    print(f"\n═══ {model.display_name or model.class_name} ═══")
    print(f"  Class:          {model.class_name}")
    print(f"  Material:       {model.material}")
    print(f"  Mass:           {model.mass_kg:.3f} kg")
    print(f"  Fragile:        {'Yes' if model.fragile else 'No'}")
    print(f"  Bounding box:   {model.bounding_box}")
    print(f"  Colors:         {model.typical_colors}")
    print()
    print(f"  ── Grasp ──")
    print(f"  Approach:       {model.approach_style}")
    print(f"  Force scale:    {model.grip_force_scale:.0%}")
    print(f"  Affordance:     {json.dumps(model.affordance, indent=2)}")
    if model.grasp_points:
        print(f"  Grasp points:   {len(model.grasp_points)}")
    print()
    print(f"  ── Statistics ──")
    print(f"  Times grasped:  {model.times_grasped}")
    rate = (f"{model.grasp_success_rate:.1%}"
            if model.times_grasped > 0 else "no data")
    print(f"  Success rate:   {rate}")
    print(f"  Date added:     {model.date_added or 'unknown'}")
    print(f"  Last seen:      {model.last_seen or 'never'}")
    print()

    if model.mesh_path:
        print(f"  Mesh:           {model.mesh_path}")
    if model.sdf_path:
        print(f"  SDF:            {model.sdf_path}")
    if model.reference_images_path:
        print(f"  Reference imgs: {model.reference_images_path}")
    if model.notes:
        print(f"  Notes:          {model.notes}")
    print()


def cmd_register(db: ObjectModelDB, args):
    """Register a new object."""
    props = {
        'mass_kg': args.mass,
        'material': args.material,
        'fragile': args.fragile,
        'grip_force_scale': args.grip_force,
    }
    if args.bounding_box:
        props['bounding_box'] = [float(x) for x in args.bounding_box.split(',')]
    if args.colors:
        props['typical_colors'] = args.colors.split(',')

    obj_id = db.register_new_object(args.name, physical_props=props)
    print(f"✅ Registered '{args.name}' (id={obj_id})")
    print(f"   To add reference images: place images in")
    print(f"   arm_planner/data/object_references/{args.name}/")


def cmd_update(db: ObjectModelDB, args):
    """Update properties of an existing object."""
    model = db.get_model(args.name)
    if model is None:
        print(f"Object '{args.name}' not found")
        return

    updates = {}
    if args.mass is not None:
        updates['mass_kg'] = args.mass
    if args.material:
        updates['material'] = args.material
    if args.grip_force is not None:
        updates['grip_force_scale'] = args.grip_force
    if args.fragile is not None:
        updates['fragile'] = int(args.fragile)

    if not updates:
        print("No updates specified")
        return

    # Build SET clause
    set_parts = [f"{k} = ?" for k in updates]
    values = list(updates.values()) + [args.name]

    with db._conn() as conn:
        conn.execute(
            f"UPDATE object_models SET {', '.join(set_parts)} "
            f"WHERE class_name = ?", values)
        conn.commit()

    print(f"✅ Updated '{args.name}': {updates}")


def cmd_stats(db: ObjectModelDB, args):
    """Show object statistics."""
    stats = db.get_stats()
    if not stats:
        print("No objects registered")
        return

    print(f"\n{'Class':>20} {'Grasps':>8} {'Success':>9} "
          f"{'Material':>12} {'Last Seen':>12}")
    print("─" * 70)

    degrading = []
    for s in stats:
        rate = (f"{s['success_rate']:.0%}"
                if s['times_grasped'] > 0 else "—")
        last = s['last_seen'][:10] if s['last_seen'] != 'never' else 'never'

        # Flag degrading objects
        flag = ''
        if s['times_grasped'] > 5 and s['success_rate'] < 0.6:
            flag = ' ⚠️'
            degrading.append(s['class_name'])

        print(f"{s['class_name']:>20} {s['times_grasped']:>8} "
              f"{rate:>9} {s['material']:>12} {last:>12}{flag}")

    if degrading:
        print(f"\n⚠️  Objects with degrading success rate (<60%):")
        for d in degrading:
            print(f"    → {d}: consider updating grasp parameters")

    print()


def cmd_export(db: ObjectModelDB, args):
    """Export object library to YAML."""
    output = args.output or 'object_library_export.yaml'
    count = db.export_library(output)
    print(f"✅ Exported {count} object(s) to {output}")


def cmd_import(db: ObjectModelDB, args):
    """Import object library from YAML."""
    if not os.path.exists(args.file):
        print(f"File not found: {args.file}")
        return

    count = db.import_library(args.file)
    print(f"✅ Imported {count} new object(s) from {args.file}")


def main():
    parser = argparse.ArgumentParser(
        prog='aria-objects',
        description='ARIA Object Model CLI')
    parser.add_argument('--db', default='', help='Database path')
    sub = parser.add_subparsers(dest='command')

    # list
    sub.add_parser('list', help='List all known objects')

    # show
    show = sub.add_parser('show', help='Show object details')
    show.add_argument('name', help='Object class name')

    # register
    reg = sub.add_parser('register', help='Register new object')
    reg.add_argument('--name', required=True, help='Class name')
    reg.add_argument('--mass', type=float, default=0.1)
    reg.add_argument('--material', default='unknown')
    reg.add_argument('--fragile', action='store_true', default=False)
    reg.add_argument('--grip-force', type=float, default=1.0)
    reg.add_argument('--bounding-box', default='')
    reg.add_argument('--colors', default='')

    # update
    upd = sub.add_parser('update', help='Update object properties')
    upd.add_argument('name', help='Object class name')
    upd.add_argument('--mass', type=float, default=None)
    upd.add_argument('--material', default='')
    upd.add_argument('--grip-force', type=float, default=None)
    upd.add_argument('--fragile', type=bool, default=None)

    # stats
    sub.add_parser('stats', help='Show object statistics')

    # export
    exp = sub.add_parser('export', help='Export to YAML')
    exp.add_argument('--output', default='')

    # import
    imp = sub.add_parser('import', help='Import from YAML')
    imp.add_argument('file', help='YAML file path')

    args = parser.parse_args()
    db = ObjectModelDB(args.db)

    handlers = {
        'list': cmd_list, 'show': cmd_show, 'register': cmd_register,
        'update': cmd_update, 'stats': cmd_stats,
        'export': cmd_export, 'import': cmd_import,
    }

    handler = handlers.get(args.command)
    if handler:
        handler(db, args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
