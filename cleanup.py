import os
import shutil

def delete_pycache(path):
    for root, dirs, files in os.walk(path):
        # Delete __pycache__ directories
        for directory in dirs:
            if directory == "__pycache__":
                dir_path = os.path.join(root, directory)
                print(f"Deleting directory: {dir_path}")
                try:
                    shutil.rmtree(dir_path)
                except OSError as e:
                    print(f"Failed to delete directory: {dir_path}\nError: {str(e)}")
        
        # Delete .pyc files
        for file in files:
            if file.endswith(".pyc"):
                file_path = os.path.join(root, file)
                print(f"Deleting file: {file_path}")
                try:
                    os.remove(file_path)
                except OSError as e:
                    print(f"Failed to delete file: {file_path}\nError: {str(e)}")

# Specify the main path
main_path = os.getcwd()

# Call the function to delete __pycache__ directories and .pyc files
delete_pycache(main_path)

