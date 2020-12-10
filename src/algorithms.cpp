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
    tsdf->setEta(.002);
    tsdf->setPaddingSize(3);
    dtype resolution = .002;
    cv::Mat mask = p_depth_img->getImage() != INF;
    tsdf->Init(p_depth_img->getImage(), mask, resolution);

    tsdf->estimateTwist(p_depth_img->getImage(), p_depth_img->getImage(), mask, mask, resolution, 50);

    auto displayer = make_shared<Display>();
    displayer->Init();
    displayer->addAxes();
    displayer->addIsoSurface(tsdf->phi, tsdf->getMinCoord(), tsdf->getDepth(), tsdf->getHeight(), tsdf->getWidth(), resolution);
    displayer->startRender();
}

