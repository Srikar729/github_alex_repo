This directory contains models for Gazebo simulation has beaker model, a volumetric flask model, and a conical flask model. Each model is defined using SDF (Simulation Description Format) and includes the necessary configuration files and mesh files.

# File Structure:

Ensure the models are structured correctly:

```
.gazebo/
  └── models/
      ├── beaker_model/
      │   ├── model.sdf
      │   ├── model.config
      │   └── meshes/
      │       └── Beaker_2 Litres.STL
      ├── volumetric_flask_model/
      │   ├── model.sdf 
      │   ├── model.config
      │   └── meshes/
      │       └── volumetric_flask.stl
      └── conical_flask/
      │   ├── model.sdf 
      │   ├── model.config
      │   └── meshes/
      │       └── conical_flask.stl
```

# Importing Models to Gazebo Ignition:

- `export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:"<MODELS_DIRECTORY>"`
- replace MODELS_DIREDCTORY to the path of the model example: (/src/ALEX/alex_gazebo/models)
- `ign gazebo empty.sdf`
    
# Command to spawn models

```bash
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "<PATH_TO_model>.sdf", name: "model_name"'
```
- replace <PATH_TO_model>.sdf to the path pf the model's sdf file exmaple:(/models/volumetric_flask_model/model.sdf)
- replace model_name to as name that you have given model.config example: (volumetric_model)

### Example
```bash
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/workspaces/hello-world-docker/src/ALEX/alex_gazebo/models/conical_flask/model.sdf", name: "conical_flask"'
```
