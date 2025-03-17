Import("env")
import os  # Import the operating system library
import re  # Import the regular expression library

def increment_version(version_string):
    """Increments a semantic version string (e.g., 1.0.0 -> 1.0.1)."""
    major, minor, patch = map(int, version_string.split("."))
    patch += 1
    return f"{major}.{minor}.{patch}"

def update_version_in_file(filepath):
    """Reads a file, updates the VERSION define, and writes it back."""
    try:
        with open(filepath, "r+") as f:  # Open in read-write mode
            content = f.read()

            # Find the #define VERSION line using a regular expression
            match = re.search(r'(#define\s+VERSION\s+)"(\d+\.\d+\.\d+)"', content)

            if match:
                current_version = match.group(2)  # Get the current version string
                new_version = increment_version(current_version)  # Increment it

                # Replace the old version with the new version
                new_content = content.replace(match.group(0), f'{match.group(1)}"{new_version}"')

                f.seek(0)  # Go back to the beginning of the file
                f.write(new_content)  # Write the modified content
                f.truncate()      # Remove any remaining old content

                print(f"Updated version in {filepath} to: {new_version}")
            else:
                print(f"Could not find VERSION define in {filepath}")

    except FileNotFoundError:
        print(f"File not found: {filepath}")
    except Exception as e:
        print(f"An error occurred: {e}")

# Get the project directory and source file path.  Adjust if needed.
project_dir = env["PROJECT_DIR"]
source_file = os.path.join(project_dir, "src", "main.cpp")

update_version_in_file(source_file)