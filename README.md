Open3D Tests
============

Building an o3d c++ example against a source build of Open3D. If you did not install o3d system-wide, you will need to provide the `CMAKE_PREFIX_PATH`:

```bash
$ cmake -DCMAKE_PREFIX_PATH=/home/plusk01/dev/o3d/Open3D/build/install ..
```

In this case, Open3D was built after running the cmake cmd

```bash
$ cmake -DCMAKE_INSTALL_PREFIX=$(pwd)/install ..
```
