//
// Created by himalaya on 12/7/20.
//

#include "display.h"

#include "display.h"
#include "algorithms.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <vtkSTLWriter.h>
#include <vtkSTLReader.h>
//#include <vtkActor.h>
#include <vtkCylinderSource.h>
//#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
//#include <vtkRenderer.h>
//#include <vtkRenderWindow.h>
//#include <vtkRenderWindowInteractor.h>
//#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkPoints.h>
//#include <vtkAssembly.h>
#include <vtkLineSource.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkAxesActor.h>
#include <vtkCommand.h>
#include <vtkCallbackCommand.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkMarchingCubes.h>
#include <vtkImageImport.h>
#include <vtkLineSource.h>
#include <iostream>

using namespace std;

vtkSmartPointer<vtkActor> Construct_lineActor(const Vec3 &p1, const Vec3 &p2)
{
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    dtype p1_arr[3] = {p1(0), p1(1), p1(2)};
    dtype p2_arr[3] = {p2(0), p2(1), p2(2)};
    line->SetPoint1(p1_arr);
    line->SetPoint2(p2_arr);
    line->Update();
    vtkSmartPointer<vtkPolyDataMapper> line_mapper= vtkSmartPointer<vtkPolyDataMapper>::New();
    line_mapper->SetInputData(line->GetOutput());
    vtkSmartPointer<vtkActor> line_actor = vtkSmartPointer<vtkActor>::New();
    line_actor->SetMapper(line_mapper);
    line_actor->GetProperty()->SetLineWidth(1.5);
    line_actor->GetProperty()->SetColor(colors->GetColor3d("Black").GetData());
    return line_actor;
}

//vtkSmartPointer<vtkActor> Render_surface(const BoundingBox &box, double level_set_val)
//{
//    auto total_num_points = box.grid3d->height * box.grid3d->width * box.grid3d->depth;
//    assert (total_num_points > 0);
//    dtype *new_phi = new dtype [total_num_points];
//    auto depth = box.grid3d->depth;
//    auto width = box.grid3d->width;
//    auto height = box.grid3d->height;
////    //// change the storing order for vtk
////#pragma omp parallel for default(none) shared(depth, width, height, new_phi, box)
////    for (IdxType z = 0; z < depth; z++) {
////        for (IdxType y = 0; y < width; y++) {
////            for (IdxType x = 0; x < height; x++) {
////                new_phi[x + y * height + z * height * width] = box.grid3d->phi[box.grid3d->Index(x, y, z)];
////            }
////        }
////    }
//    vtkSmartPointer<vtkFloatArray> phi_arr = vtkSmartPointer<vtkFloatArray>::New();
//    phi_arr->SetArray(new_phi,total_num_points, 1);
//
//    auto phi_data = vtkSmartPointer<vtkImageData>::New();
////    auto phi_data = vtkSmartPointer<vtkImageImport>::New();
//    phi_data->GetPointData()->SetScalars(phi_arr);
//    phi_data->SetDimensions(height, width, depth);
//    auto bound = box.getBoundCoord();
//    phi_data->SetOrigin(bound[0](0), bound[0](1), bound[0](2));
//    phi_data->SetSpacing(box.resolution, box.resolution, box.resolution);
//
//
//
//    auto isosurface = vtkSmartPointer<vtkMarchingCubes>::New();
//    isosurface->SetInputData(phi_data);
//    isosurface->ComputeGradientsOn();
//    isosurface->ComputeNormalsOn();
//    isosurface->ComputeScalarsOff();
//    isosurface->SetValue(0, level_set_val);
//
//    auto surface_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    surface_mapper->SetInputConnection(isosurface->GetOutputPort());
//    surface_mapper->ScalarVisibilityOn();
//
//    auto surface_actor = vtkSmartPointer<vtkActor>::New();
//    surface_actor->SetMapper(surface_mapper);
//
//    return surface_actor;
//}
//
//class vtkTimerCallback : public vtkCommand
//{
//public:
//    BoundingBox *p_box;
//    vector<ImageBase*> *p_all_cams;
//    dtype *data;
//    vtkMarchingCubes *iso;
//    vtkPoints *points;
//    vtkCellArray *vertices;
//    vtkPolyDataMapper *pt_mapper;
//    vtkTimerCallback():p_box(nullptr), p_all_cams(nullptr) {}
//    static vtkTimerCallback* New()
//    {
//        vtkTimerCallback *cb = new vtkTimerCallback;
//        return cb;
//    }
//    virtual void Execute(vtkObject* caller, unsigned long eventId, void* vtkNotUsed(callData))
//    {
//        auto *iren = dynamic_cast<vtkRenderWindowInteractor *>(caller);
//        if (this->trigger_count != 0) {
//            assert(p_box != nullptr && p_all_cams != nullptr);
//            cout << this->trigger_count++ << " callback" << endl;
//            Evolve(*p_box, *p_all_cams);
////            Transform_phi(*p_box, data);
//            iso->Modified();
//
////            pt_mapper->Modified();
//            iren->GetRenderWindow()->Render();
//        }
//        else {
//            iren->GetRenderWindow()->Render();
//            this->trigger_count++;
//        }
//
//    }
//    void Set_data(BoundingBox &box, vector<ImageBase*> &all_cams)
//    {
//        this->p_box = &box;
//        this->p_all_cams = &all_cams;
//    }
//
//private:
//    int trigger_count = 0;
//};

void KeypressCallbackFunction ( vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData) )
{
//    std::cout << "Keypress callback" << std::endl;

    auto *iren = static_cast<vtkRenderWindowInteractor*>(caller);

//    std::cout << "Pressed: " << iren->GetKeySym() << std::endl;
    char *temp_key = iren->GetKeySym();
    string key(temp_key);
    if (key == "Return") {
        iren->CreateRepeatingTimer(10);
    }
    else if (key == "p") {
        iren->DestroyTimer();
    }
}

//#if USE_NEW
//void Show_3D(vector<Camera> &all_cams, BoundingBox &box)
//{
//    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
//    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
//    vtkSmartPointer<vtkRenderWindow> renw = vtkSmartPointer<vtkRenderWindow>::New();
//    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();
//
//    for (auto &iter : all_cams) {
//        vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
//        cube->SetCenter(iter.t.at<dtype>(0, 0), iter.t.at<dtype>(0, 1), iter.t.at<dtype>(0, 2));
//        cube->SetXLength(0.5);
//        cube->SetYLength(0.5);
//        cube->SetZLength(0.5);
//        cube->Update();
//
//        vtkSmartPointer<vtkPolyDataMapper> cube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//        cube_mapper->SetInputData(cube->GetOutput());
//        vtkSmartPointer<vtkActor> cube_actor = vtkSmartPointer<vtkActor>::New();
//        cube_actor->SetMapper(cube_mapper);
//        cube_actor->GetProperty()->SetColor((colors->GetColor3d("Tomato").GetData()));
//        assembly->AddPart(cube_actor);
//    }
//    vector<Point3> bound_coord = box.Get_bound_coord();
//    assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[1]));
//    assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[2]));
//    assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[3]));
//    assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[0]));
//    assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[4]));
//    assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[5]));
//    assembly->AddPart(Construct_lineActor(bound_coord[5], bound_coord[6]));
//    assembly->AddPart(Construct_lineActor(bound_coord[6], bound_coord[7]));
//    assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[5]));
//    assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[6]));
//    assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[7]));
//    assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[7]));
//
//    // add axes
////    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
////    axes->SetTotalLength(10, 10, 10);
////    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
////    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
////    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
////    ren->AddActor(axes);
//
//    // use marching cube to render surface
//    double level_val = 0;
//    auto total_num_points = box.grid3d->height * box.grid3d->width * box.grid3d->depth;
//    assert (total_num_points > 0);
//    dtype *new_phi = new dtype [total_num_points];
//    auto depth = box.grid3d->depth;
//    auto width = box.grid3d->width;
//    auto height = box.grid3d->height;
//    Transform_phi(box, new_phi);
//
//    vtkSmartPointer<vtkFloatArray> phi_arr = vtkSmartPointer<vtkFloatArray>::New();
//    phi_arr->SetArray(new_phi,total_num_points, 0);
//
//    auto phi_data = vtkSmartPointer<vtkImageData>::New();
////    auto phi_data = vtkSmartPointer<vtkImageImport>::New();
//    phi_data->GetPointData()->SetScalars(phi_arr);
//    phi_data->SetDimensions(height, width, depth);
//    auto bound = box.Get_bound_coord();
//    phi_data->SetOrigin(bound[0].x, bound[0].y, bound[0].z);
//    phi_data->SetSpacing(box.resolution, box.resolution, box.resolution);
//
//    auto isosurface = vtkSmartPointer<vtkMarchingCubes>::New();
//    isosurface->SetInputData(phi_data);
//    isosurface->ComputeGradientsOff();
//    isosurface->ComputeNormalsOn();
//    isosurface->ComputeScalarsOff();
//    isosurface->SetValue(0, level_val);
//
//    auto surface_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    surface_mapper->SetInputConnection(isosurface->GetOutputPort());
//    surface_mapper->ScalarVisibilityOn();
//
//    vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
//    stlWriter->SetFileName("final_data.stl");
//    stlWriter->SetInputConnection(isosurface->GetOutputPort());
//
//    auto surface_actor = vtkSmartPointer<vtkActor>::New();
//    surface_actor->SetMapper(surface_mapper);
//
//    // render nonvisible point
////    auto points = vtkSmartPointer<vtkPoints>::New();
////    auto vertices = vtkSmartPointer<vtkCellArray>::New();
////    auto pt_polydata = vtkSmartPointer<vtkPolyData>::New();
////    pt_polydata->SetPoints(points);
////    pt_polydata->SetVerts(vertices);
////    auto pt_mapper = vtkSmartPointer<vtkPolyDataMappe  r>::New();
////    pt_mapper->SetInputData(pt_polydata);
////    auto pt_actor = vtkSmartPointer<vtkActor>::New();
////    const auto &grid3d = box.grid3d;
////    for (IdxType i = 0; i < box.grid3d->height; i++) {
////        for (IdxType j = 0; j < box.grid3d->width; j++) {
////            for (IdxType k = 0; k < box.grid3d->depth; k++) {
////                if (box.visibility_arr[0].psi[box.visibility_arr[0].Index(i, j, k)] < 0) {
////                    const auto &p = grid3d->coord[grid3d->Index(i, j, k)];
////                    auto id = points->InsertNextPoint(p.val);
////                    vertices->InsertNextCell(1);
////                    vertices->InsertCellPoint(id);
////                }
////            }
////        }
////    }
////    pt_actor->SetMapper(pt_mapper);
////    pt_actor->GetProperty()->SetColor(255, 0, 0);
////
////    ren->AddActor(pt_actor);
//
////    ren->AddActor(Render_surface(box, 0));
//    ren->AddActor(surface_actor);
//    ren->AddActor(assembly);
//    ren->SetBackground(colors->GetColor3d("Silver").GetData());
//    renw->AddRenderer(ren);
//    renw->SetSize(800, 800);
//    iren->SetRenderWindow(renw);
//    renw->Render();
//
//    iren->Initialize();
//
//    vtkSmartPointer<vtkCallbackCommand> keypressCallback =
//            vtkSmartPointer<vtkCallbackCommand>::New();
//    keypressCallback->SetCallback ( KeypressCallbackFunction );
//    iren->AddObserver(vtkCommand::KeyPressEvent, keypressCallback, 0);
//
//
//    auto cb = vtkSmartPointer<vtkTimerCallback>::New();
//    cb->Set_data(box, all_cams);
//    cb->data = new_phi;
//    cb->iso = isosurface;
////    cb->points = points;
////    cb->vertices = vertices;
////    cb->pt_mapper = pt_mapper;
//    iren->AddObserver(vtkCommand::TimerEvent, cb, 1.);
////    iren->CreateRepeatingTimer(1000);
//    iren->Start();
//    stlWriter->Write();
//}
//#else
//void show3D(const vector<ImageBase*> &all_cams, const BoundingBox &box)
//{
//    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
//    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
//    vtkSmartPointer<vtkRenderWindow> renw = vtkSmartPointer<vtkRenderWindow>::New();
//    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();
//
//    // test
//    vector<string> co{"Tomato", "Turquoise", "Violet", "Wheat", "White"};
//    int idx = 0;
//    for (auto &iter : all_cams) {
//        vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
//        Vec3 t = iter->getTranslation();
//        Vec3 c = -iter->getRotation().transpose() * t;
//        cube->SetCenter(c(0), c(1), c(2));
//        cube->SetXLength(0.05);
//        cube->SetYLength(0.05);
//        cube->SetZLength(0.05);
//        cube->Update();
//
//        vtkSmartPointer<vtkPolyDataMapper> cube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//        cube_mapper->SetInputData(cube->GetOutput());
//        vtkSmartPointer<vtkActor> cube_actor = vtkSmartPointer<vtkActor>::New();
//        cube_actor->SetMapper(cube_mapper);
//        cube_actor->GetProperty()->SetColor((colors->GetColor3d(co[idx++]).GetData()));
////        cube_actor->GetProperty()->
//        assembly->AddPart(cube_actor);
//    }
//    vector<Vec3> bound_coord = box.getBoundCoord();
//    // test
//    for (auto v: bound_coord) {
//        cout << v << endl;
//        cout << endl;
//    }
//    assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[1]));
//    assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[2]));
//    assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[3]));
//    assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[0]));
//    assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[4]));
//    assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[5]));
//    assembly->AddPart(Construct_lineActor(bound_coord[5], bound_coord[6]));
//    assembly->AddPart(Construct_lineActor(bound_coord[6], bound_coord[7]));
//    assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[5]));
//    assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[6]));
//    assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[7]));
//    assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[7]));
//
////     add axes
//    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
//    axes->SetTotalLength(0.1, 0.1, 0.1);
//    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
//    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
//    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
//    ren->AddActor(axes);
//
////    // render nonvisible point
////    auto points = vtkSmartPointer<vtkPoints>::New();
////    auto vertices = vtkSmartPointer<vtkCellArray>::New();
////    auto pt_polydata = vtkSmartPointer<vtkPolyData>::New();
////    pt_polydata->SetPoints(points);
////    pt_polydata->SetVerts(vertices);
////    auto pt_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
////    pt_mapper->SetInputData(pt_polydata);
////    auto pt_actor = vtkSmartPointer<vtkActor>::New();
////    const auto &grid3d = box.grid3d;
////    for (IdxType i = 0; i < box.grid3d->height; i++) {
////        for (IdxType j = 0; j < box.grid3d->width; j++) {
////            for (IdxType k = 0; k < box.grid3d->depth; k++) {
////                if (box.visibility_arr[0].psi[box.visibility_arr[0].Index(i, j, k)] < 0) {
////                    const auto &p = grid3d->coord[grid3d->Index(i, j, k)];
////                    auto id = points->InsertNextPoint(p.val);
////                    vertices->InsertNextCell(1);
////                    vertices->InsertCellPoint(id);
////                }
////            }
////        }
////    }
////    pt_actor->SetMapper(pt_mapper);
////    pt_actor->GetProperty()->SetColor(255, 0, 0);
//
////    auto line = vtkSmartPointer<vtkLineSource>::New();
////    Vec3 cam(all_cams[0].t);
////    line->SetPoint1(cam.val);
////    line->SetPoint2(grid3d->coord[grid3d->Index(grid3d->height / 4 * 3 - 2, grid3d->width - 37, grid3d->depth / 2 )].val);
////    auto line_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
////    line_mapper->SetInputConnection(line->GetOutputPort());
////    auto line_actor = vtkSmartPointer<vtkActor>::New();
////    line_actor->SetMapper(line_mapper);
////    line_actor->GetProperty()->SetColor(0, 1, 0);
////    ren->AddActor(line_actor);
//
////    ren->AddActor(pt_actor);
//
//    // render iso surface
////    ren->AddActor(Render_surface(box, 0));
//    ren->AddActor(assembly);
//    ren->SetBackground(colors->GetColor3d("Silver").GetData());
//    renw->AddRenderer(ren);
//    renw->SetSize(800, 800);
//    iren->SetRenderWindow(renw);
//    renw->Render();
//
//
//    iren->Start();
//}
//#endif

void show3D_halfline(const std::vector<const ImageBase *> &determine_cams, const std::vector<Vec3> &point3d_set, const std::vector<Vec3> &point3d_set1) {
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    vtkSmartPointer<vtkRenderer> ren = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renw = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();

    // test
    vector<string> co{"Tomato", "Turquoise", "Violet", "Wheat"};
    int idx = 0;
    for (auto &iter : determine_cams) {
        vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
        Vec3 t = iter->getTranslation();
        Vec3 c = -iter->getRotation().transpose() * t;
        cube->SetCenter(c(0), c(1), c(2));
        cube->SetXLength(0.05);
        cube->SetYLength(0.05);
        cube->SetZLength(0.05);
        cube->Update();

        vtkSmartPointer<vtkPolyDataMapper> cube_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        cube_mapper->SetInputData(cube->GetOutput());
        vtkSmartPointer<vtkActor> cube_actor = vtkSmartPointer<vtkActor>::New();
        cube_actor->SetMapper(cube_mapper);
        cube_actor->GetProperty()->SetColor((colors->GetColor3d(co[idx++]).GetData()));
//        cube_actor->GetProperty()->
        assembly->AddPart(cube_actor);
    }

    const auto &cam = determine_cams[0];
    Vec3 c = -cam->getRotation().transpose() * cam->getTranslation();
    assembly->AddPart(Construct_lineActor(c, point3d_set[0]));
    assembly->AddPart(Construct_lineActor(c, point3d_set[1]));

    const auto &cam1 = determine_cams[2];
    Vec3 c1 = -cam1->getRotation().transpose() * cam1->getTranslation();
    assembly->AddPart(Construct_lineActor(c1, point3d_set1[0]));
    assembly->AddPart(Construct_lineActor(c1, point3d_set1[1]));

    //     add axes
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetTotalLength(0.1, 0.1, 0.1);
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    ren->AddActor(axes);

    ren->AddActor(assembly);
    ren->SetBackground(colors->GetColor3d("Silver").GetData());
    renw->AddRenderer(ren);
    renw->SetSize(800, 800);
    iren->SetRenderWindow(renw);
    renw->Render();


    iren->Start();
}

Display::Display(const std::vector<ImageBase *> &all_cams): all_cams(all_cams), m_is_axes(false) {

}

Display::Display() {}

void Display::Init() {
    this->m_ren = vtkSmartPointer<vtkRenderer>::New();
    this->m_renw = vtkSmartPointer<vtkRenderWindow>::New();
    this->m_iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    this->m_colors = vtkSmartPointer<vtkNamedColors>::New();
    this->m_assembly = vtkSmartPointer<vtkAssembly>::New();
}

void Display::addCamera(int idx) {
    vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();
    Vec3 c = -this->all_cams[idx]->getRotation().transpose() * this->all_cams[idx]->getTranslation();
    cube->SetCenter(c[0], c[1], c[2]);
    cube->SetXLength(0.05);
    cube->SetYLength(0.05);
    cube->SetZLength(0.05);
    cube->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(cube->GetOutput());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(this->m_colors->GetColor3d("Tomato").GetData());
    this->m_assembly->AddPart(actor);
}

void Display::addAllCameras() {
    for (int i = 0; i < this->all_cams.size(); i++) {
        this->addCamera(i);
    }
}

void Display::addBoundingBox(const std::vector<Vec3> &bound_coord) {
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[1]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[2]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[3]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[0]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[0], bound_coord[4]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[5]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[5], bound_coord[6]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[6], bound_coord[7]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[1], bound_coord[5]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[2], bound_coord[6]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[3], bound_coord[7]));
    this->m_assembly->AddPart(Construct_lineActor(bound_coord[4], bound_coord[7]));
}

void Display::addAxes() {
    this->m_axes = vtkSmartPointer<vtkAxesActor>::New();
    this->m_axes->SetTotalLength(.1, .1, .1);
    this->m_axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    this->m_axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    this->m_axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(0.1);
    this->m_is_axes = true;
}

void Display::startRender() {
    this->m_ren->AddActor(this->m_assembly);
    if (this->m_is_axes) this->m_ren->AddActor(this->m_axes);
    this->m_ren->SetBackground(this->m_colors->GetColor3d("Silver").GetData());
    this->m_renw->AddRenderer(this->m_ren);
    this->m_renw->SetSize(800, 800);
    this->m_iren->SetRenderWindow(this->m_renw);
    this->m_renw->Render();

    this->m_iren->Start();
}

void Display::addIsoSurface(const Grid3d *grid) {
}

void Display::addIsoSurface(const std::vector<float> &phi, const Vec3 &origin,
                            int depth, int height, int width, dtype resolution,
                            const string &surface_name,
                            const string &color) {
    if (!surface_name.empty() && surface_name != "cur" && surface_name != "ref") {
        cerr << "wrong surface name\n";
        exit(EXIT_FAILURE);
    }
    if (!surface_name.empty() && iso_surfaces.count(surface_name)) {
        iso_surfaces[surface_name]->Modified();
        return;
    }
    vtkSmartPointer<vtkFloatArray> phi_arr = vtkSmartPointer<vtkFloatArray>::New();
    unsigned long total_size = depth * height * width;
    phi_arr->SetArray(const_cast<float *>(phi.data()), total_size, 1);
    auto phi_data = vtkSmartPointer<vtkImageData>::New();
    phi_data->GetPointData()->SetScalars(phi_arr);
    phi_data->SetDimensions(width, height, depth);  // len_x, len_y, len_z
    phi_data->SetOrigin(origin(0), origin(1), origin(2));
    phi_data->SetSpacing(resolution, resolution, resolution);

    auto isosurface = vtkSmartPointer<vtkMarchingCubes>::New();
    isosurface->SetInputData(phi_data);
    isosurface->ComputeGradientsOn();
    isosurface->ComputeNormalsOn();
    isosurface->ComputeScalarsOff();
    constexpr double level_set_val = 0;
    isosurface->SetValue(0, level_set_val);

    auto surface_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    surface_mapper->SetInputConnection(isosurface->GetOutputPort());
    surface_mapper->ScalarVisibilityOff();  // coloring the actor instead of data itself.

    auto surface_actor = vtkSmartPointer<vtkActor>::New();
    surface_actor->SetMapper(surface_mapper);
    if (!color.empty())
        surface_actor->GetProperty()->SetColor(m_colors->GetColor3d(color).GetData());

    this->m_assembly->AddPart(surface_actor);
    if (iso_surfaces.empty()) {
        iso_surfaces["ref"] = vtkSmartPointer<vtkMarchingCubes>::New();
        iso_surfaces["ref"] = isosurface;
    }
    else {
        iso_surfaces["cur"] = vtkSmartPointer<vtkMarchingCubes>::New();
        iso_surfaces["cur"] = isosurface;
    }
}