#!/usr/bin/env python3
"""
Person Manager - CLI for managing person entities

Provides a command-line interface for CRUD operations on persons in the registry.
"""

import sys
from pathlib import Path
from person_registry import PersonRegistry


class PersonManager:
    """CLI manager for person registry operations."""
    
    def __init__(self):
        """Initialize person manager with registry."""
        self.registry = PersonRegistry()
    
    def list_persons(self):
        """List all registered persons with their details."""
        persons = self.registry.list_persons()
        
        if not persons:
            print("\n❌ No persons registered yet.")
            return
        
        print("\n" + "="*70)
        print("                     Registered Persons")
        print("="*70)
        
        for person in persons:
            print(f"\n📝 {person['display_name']}")
            print(f"   ID: {person['id']}")
            print(f"   Created: {person['created_at']}")
            print(f"   Face Model: {'✓' if person['face_model_path'] else '✗'}")
            print(f"   Gesture Model: {'✓' if person['gesture_model_path'] else '✗'}")
        
        print("\n" + "="*70)
    
    def view_person_details(self):
        """View detailed information for a specific person."""
        name = input("\nEnter person name: ").strip()
        if not name:
            print("❌ Name cannot be empty.")
            return
        
        person = self.registry.get_person(name)
        if not person:
            print(f"\n❌ Person '{name}' not found.")
            return
        
        print("\n" + "="*70)
        print(f"                  Person Details: {person['display_name']}")
        print("="*70)
        print(f"\nID: {person['id']}")
        print(f"Name: {person['name']}")
        print(f"Display Name: {person['display_name']}")
        print(f"Created: {person['created_at']}")
        print(f"Updated: {person['updated_at']}")
        print(f"\nFace Model: {person['face_model_path'] or 'Not trained'}")
        print(f"Gesture Model: {person['gesture_model_path'] or 'Not trained'}")
        print(f"Google Account: {person['google_account'] or 'Not linked'}")
        print("\n" + "="*70)
    
    def create_person(self):
        """Create a new person in the registry."""
        print("\n" + "="*70)
        print("                     Create New Person")
        print("="*70)
        
        name = input("\nEnter person name: ").strip()
        if not name:
            print("❌ Name cannot be empty.")
            return
        
        try:
            person_id = self.registry.register_person(name)
            print(f"\n✅ Person '{name}' registered successfully!")
            print(f"   ID: {person_id}")
            print(f"\n💡 Next steps:")
            print(f"   - Train face recognition (train_manager.py → option 1)")
            print(f"   - Train gesture recognition (train_manager.py → option 8)")
        except ValueError as e:
            print(f"\n❌ Error: {e}")
    
    def delete_person(self):
        """Delete a person from the registry."""
        name = input("\nEnter person name to delete: ").strip()
        if not name:
            print("❌ Name cannot be empty.")
            return
        
        person = self.registry.get_person(name)
        if not person:
            print(f"\n❌ Person '{name}' not found.")
            return
        
        # Show what will be deleted
        print("\n" + "="*70)
        print(f"                 ⚠️  Delete Person: {person['display_name']}")
        print("="*70)
        print(f"\nThis will remove '{person['display_name']}' from the person registry.")
        print(f"\n⚠️  WARNING: This does NOT delete:")
        print(f"   - Face recognition model: {person['face_model_path'] or 'N/A'}")
        print(f"   - Gesture recognition model: {person['gesture_model_path'] or 'N/A'}")
        print(f"   - Training images")
        print(f"\nTo fully delete a person, use train_manager.py → option 7 (delete person)")
        print("\n" + "="*70)
        
        confirm = input(f"\nAre you sure you want to remove '{name}' from registry? (yes/no): ").strip().lower()
        if confirm not in ['yes', 'y']:
            print("❌ Deletion cancelled.")
            return
        
        self.registry.delete_person(person['id'])
        print(f"\n✅ Person '{name}' removed from registry.")
    
    def migrate_existing_models(self):
        """Auto-migrate existing face and gesture models."""
        print("\n" + "="*70)
        print("              Auto-Migrate Existing Models")
        print("="*70)
        print("\nThis will scan for existing face and gesture recognition models")
        print("and automatically register them in the person registry.")
        
        confirm = input("\nProceed with migration? (yes/no): ").strip().lower()
        if confirm not in ['yes', 'y']:
            print("❌ Migration cancelled.")
            return
        
        print("\n🔄 Scanning for existing models...")
        results = self.registry.auto_migrate()
        
        print("\n" + "="*70)
        print("                  Migration Results")
        print("="*70)
        print(f"\n✅ Migrated: {results['migrated']} model(s)")
        print(f"⏭️  Skipped: {results['skipped']} model(s) (already registered)")
        print(f"❌ Errors: {results['errors']} model(s)")
        
        if results['migrated'] > 0:
            print("\n✅ Migration complete! Use option 1 to view registered persons.")
        else:
            print("\n💡 No new models found to migrate.")
        
        print("\n" + "="*70)
    
    def main_menu(self):
        """Display main menu and handle user input."""
        while True:
            print("\n" + "="*70)
            print("                   Person Registry Manager")
            print("="*70)
            print("\n[1] List all persons")
            print("[2] View person details")
            print("[3] Create new person")
            print("[4] Delete person (registry only)")
            print("[5] Migrate existing models")
            print("[0] Exit")
            print("\n" + "="*70)
            
            choice = input("\nEnter choice: ").strip()
            
            if choice == '1':
                self.list_persons()
            elif choice == '2':
                self.view_person_details()
            elif choice == '3':
                self.create_person()
            elif choice == '4':
                self.delete_person()
            elif choice == '5':
                self.migrate_existing_models()
            elif choice == '0':
                print("\n👋 Exiting person manager.")
                break
            else:
                print("\n❌ Invalid choice. Please try again.")
            
            input("\nPress ENTER to continue...")


def main():
    """Main entry point."""
    try:
        manager = PersonManager()
        manager.main_menu()
    except KeyboardInterrupt:
        print("\n\n👋 Interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()

