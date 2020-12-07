//
// Created by himalaya on 12/7/20.
//

#include "algorithms.h"
#include "display.h"

using namespace std;
void testAll() {
    auto p_depth_img = std::make_shared<DepthImage>();
    p_depth_img->loadImage("../res/Synthetic_Bunny_Circle/depth_000000.exr");
    p_depth_img->showImage();
    p_depth_img->loadIntrinsic("..//res/Synthetic_Bunny_Circle/intrinsic.txt");

    auto tsdf = make_shared<TSDF>();
    tsdf->setIntrinsic(p_depth_img->getIntrinsic());
    tsdf->setDelta(0.002);
    tsdf->setEta(.001);
    tsdf->setPaddingSize(3);
    dtype resolution = .002;
    tsdf->Init(p_depth_img->getImage(), resolution);

    auto displayer = make_shared<Display>();
    displayer->Init();
    displayer->addIsoSurface(tsdf->phi, tsdf->getMinCoord(), tsdf->getDepth(), tsdf->getHeight(), tsdf->getWidth(), resolution);

    // test displaying two isosurface
    vector<dtype> test_phi;
    auto p_depth_img2 = std::make_shared<DepthImage>();
    p_depth_img2->loadImage("../res/Synthetic_Bunny_Circle/depth_000001.exr");
    tsdf->computeAnotherPhi(p_depth_img2->getImage(), test_phi);
    displayer->addIsoSurface(test_phi, tsdf->getMinCoord(), tsdf->getDepth(), tsdf->getHeight(), tsdf->getWidth(), resolution, "Silver");

    displayer->addAxes();
    displayer->startRender();
}

