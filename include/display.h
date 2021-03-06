//
// Created by himalaya on 12/7/20.
//

#ifndef SDF_2_SDF_DISPLAY_H
#define SDF_2_SDF_DISPLAY_H
#include "base.h"
#include "camera.h"
#include "grid3d.h"
#include <vtkAssembly.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>
#include <vtkNamedColors.h>
#include <vtkAxesActor.h>
#include <vtkMarchingCubes.h>
#include <vtkCamera.h>
#include <unordered_map>    // iso_surfaces
#include <thread>       // eventloop

class Display {
    // should be const
//    BoundingBox *__box;
    // should be const
    vtkSmartPointer<vtkNamedColors> m_colors;
    vtkSmartPointer<vtkAxesActor> m_axes;
    bool m_is_axes;
    vtkSmartPointer<vtkRenderer> m_ren;
    vtkSmartPointer<vtkRenderWindow> m_renw;
    vtkSmartPointer<vtkRenderWindowInteractor> m_iren;
    vtkSmartPointer<vtkAssembly> m_assembly;
    std::unordered_map<std::string, vtkMarchingCubes *> iso_surfaces;

public:
    std::vector<ImageBase *> all_cams;
    Display(const std::vector<ImageBase *> &all_cams);
    Display();
    void Init();
    void addCamera(int idx);
    void addAllCameras();
    void addBoundingBox(const std::vector<Vec3> &bound_coord);
    void addIsoSurface(const Grid3d *grid);
    void addIsoSurface(const std::vector<float> &phi, const Vec3 &origin,
                       int depth, int height, int width, dtype resolution,
                       const std::string &surface_name = "",
                       const std::string &color="");
    void updateIsoSurface(const std::vector<float> &phi, const Vec3 &origin,
                          int depth, int height, int width, dtype resolution,
                          const std::string &surface_name, const std::string  &color = "Silver");
    void addAxes();
    void startRender();
    void startInteractor();
    void render();
    void close();
    void eventLoop(TSDF *tsdf);

};

void show3D(const std::vector<ImageBase*> &all_cams);
#endif //SDF_2_SDF_DISPLAY_H
