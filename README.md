In order to compile, the vision server needs 
the vision_filter library, the vision_lib library and vitals library.
Before doing anything, pull the lastest version of every repo.

Then do those command:
- cmake ./
- make -j8
- sudo make install

in the following folders, IN THE SAME ORDER AS HERE:
vitals
sonia_lib
sonia_filter

NOTE:
Be sure to have followed the Installation OpenCv page on the wiki 
AND to have set your SONIA_WORKSPACE_ROOT environnement variable.
