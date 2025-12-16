# URDF Validation Tool

This tool provides comprehensive validation of URDF (Unified Robot Description Format) files to ensure they meet the requirements for Physical AI and humanoid robotics applications.

## Purpose
- Validate URDF file syntax and structure
- Check for common errors in robot definition
- Verify physical properties and joint limits
- Ensure compatibility with simulation and control systems
- Provide detailed reports of issues found

## Inputs
- URDF file path
- Validation configuration parameters
- Robot-specific requirements and constraints

## Outputs
- Validation report with issues found
- Recommendations for fixes
- URDF quality score
- Compliance status with standards

## Implementation

```python
#!/usr/bin/env python3
"""
URDF Validation Tool

This tool validates URDF files for Physical AI and robotics applications.
"""

import xml.etree.ElementTree as ET
import argparse
import sys
import os
from typing import List, Dict, Tuple, Optional
import math
import yaml

class URDFValidator:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.info = []
        
        # Common robot joint limits (for reference)
        self.joint_limits = {
            'humanoid': {
                'hip': {'min': -1.57, 'max': 1.57},      # -90 to 90 degrees
                'knee': {'min': 0, 'max': 2.35},         # 0 to 135 degrees
                'ankle': {'min': -0.78, 'max': 0.78},    # -45 to 45 degrees
                'shoulder': {'min': -2.35, 'max': 2.35}, # -135 to 135 degrees
                'elbow': {'min': 0, 'max': 2.35},        # 0 to 135 degrees
            }
        }
        
        # Required elements for valid URDF
        self.required_elements = {
            'robot': ['name'],
            'link': ['name'],
            'joint': ['name', 'type'],
            'parent': ['link'],
            'child': ['link']
        }

    def validate_urdf_file(self, urdf_path: str, robot_type: str = 'general') -> Dict[str, any]:
        """
        Validate a URDF file and return detailed report
        """
        # Reset validation results
        self.errors = []
        self.warnings = []
        self.info = []
        
        if not os.path.exists(urdf_path):
            self.errors.append(f"URDF file does not exist: {urdf_path}")
            return self._generate_report()
        
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
        except ET.ParseError as e:
            self.errors.append(f"XML parsing error: {str(e)}")
            return self._generate_report()
        except Exception as e:
            self.errors.append(f"Error loading URDF: {str(e)}")
            return self._generate_report()
        
        # Validate root element
        if root.tag != 'robot':
            self.errors.append("Root element must be 'robot'")
            return self._generate_report()
        
        robot_name = root.get('name', '')
        if not robot_name:
            self.errors.append("Robot must have a name attribute")
        
        # Validate the complete URDF structure
        self._validate_robot_structure(root)
        self._validate_links(root)
        self._validate_joints(root)
        self._validate_materials(root)
        self._validate_gazebo_extensions(root)
        
        # Check for robot-specific issues
        if robot_type == 'humanoid':
            self._validate_humanoid_specifics(root)
        
        return self._generate_report()

    def _validate_robot_structure(self, root):
        """Validate basic structure of robot element"""
        robot_name = root.get('name', '')
        if not robot_name:
            self.errors.append("Robot element missing 'name' attribute")
        
        # Check for duplicate names
        link_names = []
        joint_names = []
        
        for link in root.findall('link'):
            name = link.get('name')
            if name in link_names:
                self.errors.append(f"Duplicate link name found: {name}")
            else:
                link_names.append(name)
        
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name in joint_names:
                self.errors.append(f"Duplicate joint name found: {name}")
            else:
                joint_names.append(name)

    def _validate_links(self, root):
        """Validate all link elements"""
        for link in root.findall('link'):
            link_name = link.get('name')
            
            # Check if link has at least one visual or collision element
            visual_count = len(link.findall('visual'))
            collision_count = len(link.findall('collision'))
            
            if visual_count == 0 and collision_count == 0:
                self.warnings.append(f"Link '{link_name}' has no visual or collision elements")
            
            # Validate inertial properties
            inertial_elem = link.find('inertial')
            if inertial_elem is not None:
                self._validate_inertial(inertial_elem, link_name)
            else:
                self.warnings.append(f"Link '{link_name}' has no inertial properties - may cause physics issues")
            
            # Validate visual elements
            for visual in link.findall('visual'):
                self._validate_visual(visual, link_name)
            
            # Validate collision elements
            for collision in link.findall('collision'):
                self._validate_collision(collision, link_name)

    def _validate_inertial(self, inertial_elem, link_name):
        """Validate inertial properties of a link"""
        mass_elem = inertial_elem.find('mass')
        if mass_elem is None:
            self.errors.append(f"Link '{link_name}' inertial missing mass element")
            return
        
        mass = mass_elem.get('value')
        if mass is None:
            self.errors.append(f"Link '{link_name}' mass element missing value attribute")
            return
        
        try:
            mass_val = float(mass)
            if mass_val <= 0:
                self.errors.append(f"Link '{link_name}' has non-positive mass: {mass_val}")
        except ValueError:
            self.errors.append(f"Link '{link_name}' has invalid mass value: {mass}")
        
        # Check inertia matrix
        inertia_elem = inertial_elem.find('inertia')
        if inertia_elem is not None:
            ixx = float(inertia_elem.get('ixx', 0))
            iyy = float(inertia_elem.get('iyy', 0))
            izz = float(inertia_elem.get('izz', 0))
            ixy = float(inertia_elem.get('ixy', 0))
            ixz = float(inertia_elem.get('ixz', 0))
            iyz = float(inertia_elem.get('iyz', 0))
            
            # Basic plausibility checks
            if ixx < 0 or iyy < 0 or izz < 0:
                self.errors.append(f"Link '{link_name}' has negative diagonal inertia values")
            
            # Check if inertia values are reasonable (rough heuristic)
            if mass_val > 0 and max(ixx, iyy, izz) > mass_val:
                self.warnings.append(f"Link '{link_name}' inertia values seem large relative to mass")

    def _validate_visual(self, visual_elem, link_name):
        """Validate visual element of a link"""
        # Check for geometry
        geometry_elem = visual_elem.find('geometry')
        if geometry_elem is None:
            self.errors.append(f"Link '{link_name}' visual element missing geometry")
            return
        
        # Check if it has at least one primitive geometry type
        geometry_types = ['box', 'cylinder', 'sphere', 'mesh']
        geometry_found = False
        
        for geom_type in geometry_types:
            if geometry_elem.find(geom_type) is not None:
                geometry_found = True
                break
        
        if not geometry_found:
            self.errors.append(f"Link '{link_name}' visual geometry has no recognized shape")
        
        # Check origin transformation
        origin_elem = visual_elem.find('origin')
        if origin_elem is not None:
            xyz = origin_elem.get('xyz', '0 0 0').split()
            rpy = origin_elem.get('rpy', '0 0 0').split()
            
            if len(xyz) != 3:
                self.errors.append(f"Link '{link_name}' visual origin xyz must have 3 values")
            if len(rpy) != 3:
                self.errors.append(f"Link '{link_name}' visual origin rpy must have 3 values")

    def _validate_collision(self, collision_elem, link_name):
        """Validate collision element of a link"""
        # Check for geometry
        geometry_elem = collision_elem.find('geometry')
        if geometry_elem is None:
            self.errors.append(f"Link '{link_name}' collision element missing geometry")
            return
        
        # Check if it has at least one primitive geometry type
        geometry_types = ['box', 'cylinder', 'sphere', 'mesh']
        geometry_found = False
        
        for geom_type in geometry_types:
            if geometry_elem.find(geom_type) is not None:
                geometry_found = True
                break
        
        if not geometry_found:
            self.errors.append(f"Link '{link_name}' collision geometry has no recognized shape")

    def _validate_joints(self, root):
        """Validate all joint elements"""
        for joint in root.findall('joint'):
            joint_name = joint.get('name')
            joint_type = joint.get('type')
            
            if joint_type not in ['revolute', 'continuous', 'prismatic', 'fixed', 'floating', 'planar']:
                self.errors.append(f"Joint '{joint_name}' has invalid type: {joint_type}")
            
            # Check for parent and child elements
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')
            
            if parent_elem is None:
                self.errors.append(f"Joint '{joint_name}' missing parent element")
            elif parent_elem.get('link') is None:
                self.errors.append(f"Joint '{joint_name}' parent element missing link attribute")
            
            if child_elem is None:
                self.errors.append(f"Joint '{joint_name}' missing child element")
            elif child_elem.get('link') is None:
                self.errors.append(f"Joint '{joint_name}' child element missing link attribute")
            
            # Check joint limits
            if joint_type in ['revolute', 'prismatic']:
                limit_elem = joint.find('limit')
                if limit_elem is None:
                    self.warnings.append(f"Joint '{joint_name}' ({joint_type}) missing limit element")
                else:
                    self._validate_joint_limits(limit_elem, joint_name, joint_type)

    def _validate_joint_limits(self, limit_elem, joint_name, joint_type):
        """Validate joint limit specifications"""
        lower = limit_elem.get('lower')
        upper = limit_elem.get('upper')
        effort = limit_elem.get('effort')
        velocity = limit_elem.get('velocity')
        
        if joint_type == 'revolute':
            # For revolute joints, check angle limits
            if lower is not None and upper is not None:
                try:
                    lower_val = float(lower)
                    upper_val = float(upper)
                    
                    if lower_val >= upper_val:
                        self.errors.append(f"Joint '{joint_name}' lower limit >= upper limit: {lower_val} >= {upper_val}")
                    
                    if abs(upper_val - lower_val) > 2 * math.pi:
                        self.warnings.append(f"Joint '{joint_name}' has limits larger than 2π - consider if this should be continuous")
                        
                except ValueError:
                    self.errors.append(f"Joint '{joint_name}' has invalid limit values: {lower}, {upper}")
        
        elif joint_type == 'prismatic':
            # For prismatic joints, check linear limits
            if lower is not None and upper is not None:
                try:
                    lower_val = float(lower)
                    upper_val = float(upper)
                    
                    if lower_val >= upper_val:
                        self.errors.append(f"Joint '{joint_name}' lower limit >= upper limit: {lower_val} >= {upper_val}")
                        
                except ValueError:
                    self.errors.append(f"Joint '{joint_name}' has invalid limit values: {lower}, {upper}")
        
        # Check effort limit
        if effort is not None:
            try:
                effort_val = float(effort)
                if effort_val <= 0:
                    self.errors.append(f"Joint '{joint_name}' has non-positive effort limit: {effort_val}")
            except ValueError:
                self.errors.append(f"Joint '{joint_name}' has invalid effort value: {effort}")
        
        # Check velocity limit
        if velocity is not None:
            try:
                velocity_val = float(velocity)
                if velocity_val <= 0:
                    self.errors.append(f"Joint '{joint_name}' has non-positive velocity limit: {velocity_val}")
            except ValueError:
                self.errors.append(f"Joint '{joint_name}' has invalid velocity value: {velocity}")

    def _validate_materials(self, root):
        """Validate material definitions"""
        for material in root.findall('material'):
            mat_name = material.get('name')
            
            # Check for either color or texture
            color_elem = material.find('color')
            texture_elem = material.find('texture')
            
            if color_elem is None and texture_elem is None:
                self.warnings.append(f"Material '{mat_name}' has no color or texture definition")
            
            if color_elem is not None:
                rgba = color_elem.get('rgba')
                if rgba is None:
                    self.errors.append(f"Material '{mat_name}' color element missing rgba attribute")
                else:
                    rgba_values = rgba.split()
                    if len(rgba_values) != 4:
                        self.errors.append(f"Material '{mat_name}' color rgba must have 4 values (r g b a)")

    def _validate_gazebo_extensions(self, root):
        """Validate Gazebo-specific extensions"""
        for gazebo in root.findall('gazebo'):
            # Check for plugin definitions
            for plugin in gazebo.findall('plugin'):
                name = plugin.get('name')
                filename = plugin.get('filename')
                
                if not name:
                    self.warnings.append("Gazebo plugin missing name attribute")
                
                if not filename:
                    self.warnings.append("Gazebo plugin missing filename attribute")

    def _validate_humanoid_specifics(self, root):
        """Validate humanoid robot specific requirements"""
        links = [link.get('name') for link in root.findall('link')]
        joints = [(j.get('name'), j.get('type')) for j in root.findall('joint')]
        
        # Check for basic humanoid components
        humanoid_parts = ['head', 'torso', 'pelvis', 'left_leg', 'right_leg', 'left_arm', 'right_arm']
        found_parts = [part for part in humanoid_parts if any(part in link.lower() for link in links)]
        
        missing_parts = [part for part in humanoid_parts if part not in found_parts]
        if missing_parts:
            self.warnings.append(f"Humanoid robot missing parts: {missing_parts}")
        
        # Check for typical humanoid joint types
        humanoid_joints = [j for j in joints if any(part in j[0].lower() for part in ['hip', 'knee', 'ankle', 'shoulder', 'elbow', 'wrist'])]
        
        if not humanoid_joints:
            self.warnings.append("Humanoid robot appears to be missing typical joint types (hip, knee, shoulder, etc.)")

    def _generate_report(self) -> Dict[str, any]:
        """Generate validation report"""
        total_issues = len(self.errors) + len(self.warnings)
        
        # Calculate quality score (simplified: perfect score is 100, each error reduces by 10, warning by 2)
        max_score = 100
        score_reduction = len(self.errors) * 10 + len(self.warnings) * 2
        quality_score = max(0, max_score - score_reduction)
        
        compliance = "FAIL" if self.errors else ("PARTIAL" if self.warnings else "PASS")
        
        return {
            'errors': self.errors,
            'warnings': self.warnings,
            'info': self.info,
            'quality_score': quality_score,
            'compliance': compliance,
            'total_issues': total_issues,
            'error_count': len(self.errors),
            'warning_count': len(self.warnings)
        }

def main():
    parser = argparse.ArgumentParser(description='URDF Validation Tool for Physical AI')
    parser.add_argument('urdf_file', help='Path to URDF file to validate')
    parser.add_argument('--robot-type', choices=['general', 'humanoid', 'manipulator'], 
                       default='general', help='Type of robot for specialized validation')
    parser.add_argument('--format', choices=['text', 'json', 'yaml'], 
                       default='text', help='Output format')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.urdf_file):
        print(f"Error: File does not exist: {args.urdf_file}", file=sys.stderr)
        sys.exit(1)
    
    validator = URDFValidator()
    report = validator.validate_urdf_file(args.urdf_file, args.robot_type)
    
    if args.format == 'json':
        import json
        print(json.dumps(report, indent=2))
    elif args.format == 'yaml':
        import yaml
        print(yaml.dump(report, default_flow_style=False))
    else:  # text format
        print("=" * 60)
        print(f"URDF VALIDATION REPORT: {args.urdf_file}")
        print("=" * 60)
        print(f"Robot Type: {args.robot_type}")
        print(f"Quality Score: {report['quality_score']}/100")
        print(f"Compliance: {report['compliance']}")
        print(f"Total Issues: {report['total_issues']} (Errors: {report['error_count']}, Warnings: {report['warning_count']})")
        print()
        
        if report['errors']:
            print("ERRORS:")
            for i, error in enumerate(report['errors'], 1):
                print(f"  {i}. {error}")
            print()
        
        if report['warnings']:
            print("WARNINGS:")
            for i, warning in enumerate(report['warnings'], 1):
                print(f"  {i}. {warning}")
            print()
        
        if not report['errors'] and not report['warnings']:
            print("No issues found. URDF appears to be valid!")
        
        print("=" * 60)

if __name__ == '__main__':
    main()
```

## Usage Examples

### Command line usage
```
# Basic validation
python3 urdf_validator.py path/to/robot.urdf

# Validate as humanoid robot
python3 urdf_validator.py path/to/humanoid.urdf --robot-type humanoid

# Output in JSON format
python3 urdf_validator.py path/to/robot.urdf --format json

# Output in YAML format
python3 urdf_validator.py path/to/robot.urdf --format yaml
```

### Python API usage
```
from urdf_validator import URDFValidator

validator = URDFValidator()
report = validator.validate_urdf_file('path/to/robot.urdf', 'humanoid')

print(f"Quality Score: {report['quality_score']}")
print(f"Errors: {report['errors']}")
print(f"Warnings: {report['warnings']}")
```

## Integration Notes
- This tool should be integrated into the robot development workflow
- Can be used as a pre-commit hook to validate URDF files
- Should be run before simulation to catch issues early
- Quality score can be used for CI/CD pipelines
- Integration with IDE could provide real-time validation