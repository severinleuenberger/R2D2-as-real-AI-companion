#!/usr/bin/env python3
"""Test script for person registry system."""

from person_registry import PersonRegistry

def main():
    print("="*70)
    print("Testing Person Registry System")
    print("="*70)
    
    # Initialize registry
    print("\n1. Initializing PersonRegistry...")
    registry = PersonRegistry()
    print(f"   ✓ Database: {registry.db_path}")
    
    # Test auto-migration
    print("\n2. Running auto-migration...")
    results = registry.auto_migrate()
    print(f"   Migrated: {results['migrated']}")
    print(f"   Skipped: {results['skipped']}")
    print(f"   Errors: {results['errors']}")
    
    # List persons
    print("\n3. Listing registered persons...")
    persons = registry.list_persons()
    if persons:
        for person in persons:
            print(f"\n   📝 {person['display_name']}")
            print(f"      ID: {person['id']}")
            print(f"      Face Model: {'✓' if person['face_model_path'] else '✗'}")
            print(f"      Gesture Model: {'✓' if person['gesture_model_path'] else '✗'}")
    else:
        print("   No persons registered yet.")
    
    # Test get_person
    if persons:
        print("\n4. Testing get_person()...")
        first_person = persons[0]
        person_detail = registry.get_person(first_person['name'])
        if person_detail:
            print(f"   ✓ Retrieved: {person_detail['display_name']}")
            print(f"      Face: {person_detail['face_model_path'] or 'N/A'}")
            print(f"      Gesture: {person_detail['gesture_model_path'] or 'N/A'}")
    
    print("\n" + "="*70)
    print("✅ All tests completed!")
    print("="*70)

if __name__ == '__main__':
    main()

