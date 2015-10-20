# Provider Vision Documentation

This documentation aims to provide a conceptual overview of the vision provider.
This will not describe the logic of the different code parts but this will 
rather provide an entry point on how to use this software as a user.

If you want to have a more detailed documentation, please refer to our Doxygen
server.

This documentation is separated into two parts:
- A description of the conception of the software and the overall architecture.
  The ROS details like services and messages.
- The classes documentation that provides a synopsys of every classes and an
  exemple of usage for each of them.

### Design

* [ROS Interface](ros.md)
* [Architecture](architecture.md)

### Classes : `media`

* [camera_configuration](media/camera_configuration.md)
* [configuration_handler](media/configuration_handler.md)
* [media_streamer](media/media_streamer.md)

### Classes : `media/camera`

* [base_camera](media/camera/base_camera.md)

### Classes : `media/context`

* [base_context](media/context/base_context.md)

### Classes : `proc`

* [detection_task](proc/detection_task.md)
* [filterchain](proc/filterchain.md)

### Classes : `server`

* [detection_task_manager](server/detection_task_manager.md)
* [filterchain_manager](server/filterchain_manager.md)
* [media_manager](server/media_manager.md)
* [vision_server](server/vision_server.md)

