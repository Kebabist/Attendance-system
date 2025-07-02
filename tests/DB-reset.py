import os
import shutil
import glob

# Define the paths to your files and folders based on your server.py configuration
# These should match the constants in your server.py or be passed as arguments
DB_FILE_PATH = 'attendance_system.db'
SPECIFIC_JSON_FILES = ['students.json', 'devices.json']
# If you have other JSON files in the same directory as your script that need to be deleted:
OTHER_JSON_PATTERN = '*.json' # This will find all .json files in the current directory
FOLDERS_TO_DELETE = ['firmware'] # Example: ['firmware', 'logs_folder']

def reset_all_data(db_path, specific_jsons, other_json_pattern, folders):
    """
    Deletes the specified database file, JSON files, and folders.

    Args:
        db_path (str): Path to the database file.
        specific_jsons (list): A list of paths to specific JSON files to delete.
        other_json_pattern (str): A glob pattern to find other JSON files to delete 
                                  (e.g., '*.json' for all JSON files in the script's directory).
                                  Set to None or empty string if not needed.
        folders (list): A list of paths to folders to delete.

    WARNING: This function permanently deletes files and folders.
    Ensure you have backups and understand what will be deleted.
    """
    print("WARNING: This will permanently delete data.")
    print("Attempting to delete the following:")
    print(f"- Database: {db_path}")
    for f in specific_jsons:
        print(f"- Specific JSON: {f}")
    if other_json_pattern:
        print(f"- Other JSONs matching: {other_json_pattern} in the current directory")
    for folder in folders:
        print(f"- Folder: {folder} and all its contents")
    
    # --- Delete Database File ---
    try:
        if os.path.exists(db_path):
            os.remove(db_path)
            print(f"SUCCESS: Deleted database file '{db_path}'")
        else:
            print(f"INFO: Database file '{db_path}' not found.")
    except OSError as e:
        print(f"ERROR: Could not delete database file '{db_path}': {e}")

    # --- Delete Specific JSON Files ---
    for json_file in specific_jsons:
        try:
            if os.path.exists(json_file):
                os.remove(json_file)
                print(f"SUCCESS: Deleted JSON file '{json_file}'")
            else:
                print(f"INFO: JSON file '{json_file}' not found.")
        except OSError as e:
            print(f"ERROR: Could not delete JSON file '{json_file}': {e}")

    # --- Delete Other JSON Files based on pattern (in current directory) ---
    if other_json_pattern:
        # This glob pattern works relative to the current working directory
        # If your JSON files are elsewhere, adjust the pattern accordingly (e.g., "path/to/json_dir/*.json")
        other_json_files_found = glob.glob(other_json_pattern)
        # Ensure we don't re-delete the specific ones if they match the pattern and are in the CWD
        # This can be refined if specific_jsons contains full paths vs just basenames
        files_to_delete_by_pattern = [
            f for f in other_json_files_found if os.path.basename(f) not in [os.path.basename(sf) for sf in specific_jsons]
        ]

        if not files_to_delete_by_pattern and other_json_files_found:
             print(f"INFO: Other JSON files matching '{other_json_pattern}' were already listed in specific_jsons or not found.")
        elif not files_to_delete_by_pattern:
            print(f"INFO: No other JSON files matching '{other_json_pattern}' found to delete.")


        for json_file in files_to_delete_by_pattern:
            try:
                if os.path.exists(json_file): # Redundant check if glob worked, but safe
                    os.remove(json_file)
                    print(f"SUCCESS: Deleted other JSON file '{json_file}' (matched by pattern)")
                # No 'else' needed here as glob.glob only returns existing files
            except OSError as e:
                print(f"ERROR: Could not delete other JSON file '{json_file}': {e}")
    
    # --- Delete Folders ---
    for folder_path in folders:
        try:
            if os.path.exists(folder_path):
                shutil.rmtree(folder_path)
                print(f"SUCCESS: Deleted folder '{folder_path}' and all its contents")
            else:
                print(f"INFO: Folder '{folder_path}' not found.")
        except OSError as e:
            print(f"ERROR: Could not delete folder '{folder_path}': {e}")
            
    print("Reset process finished.")

# Example of how to call this function:
if __name__ == '__main__':
    # IMPORTANT: UNCOMMENTING AND RUNNING THE FOLLOWING LINES WILL DELETE DATA.
    # MAKE ABSOLUTELY SURE YOU WANT TO DO THIS.
    # CONSIDER ADDING AN INPUT CONFIRMATION.
    
    # print("\n--- Example Call ---")
    confirmation = input("Are you absolutely sure you want to delete all data? Type 'YES' to confirm: ")
    if confirmation == 'YES':
        reset_all_data(
            db_path=DB_FILE_PATH,
            specific_jsons=SPECIFIC_JSON_FILES,
            other_json_pattern=OTHER_JSON_PATTERN, # Set to None or "" if you don't want to delete other JSONs by pattern
            folders=FOLDERS_TO_DELETE
        )
    else:
        print("Data deletion cancelled.")
    pass # Keep the example commented out by default