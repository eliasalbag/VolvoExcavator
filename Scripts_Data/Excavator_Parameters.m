clear all, close all, clc
%Dimensions in m

%Links
Excavator_Param.Link.BaseRadius = 15e-3;
Excavator_Param.Link.BaseLength = 250e-3;

Excavator_Param.Link.BomRadius = 15e-3;
Excavator_Param.Link.BomLength = 200e-3;

Excavator_Param.Link.StickRadius = 15e-3;
Excavator_Param.Link.StickLength = 150e-3;



%Base
Excavator_Param.Base.CutoutWidth = 25e-3;
Excavator_Param.Base.Width = 250e-3;
Excavator_Param.Base.InnerWidth = Excavator_Param.Base.Width - 2*Excavator_Param.Base.CutoutWidth;

%Bom
Excavator_Param.Bom.Width = 200e-3;
Excavator_Param.Bom.CutoutWidth = 25e-3;
Excavator_Param.Bom.InnerWidth = Excavator_Param.Bom.Width - 2*Excavator_Param.Bom.CutoutWidth;

%Stick
Excavator_Param.Stick.Width = 150e-3;
Excavator_Param.Stick.CutoutWidth = 25e-3;
Excavator_Param.Stick.InnerWidth = Excavator_Param.Stick.Width - 2*Excavator_Param.Stick.CutoutWidth;

%Linkage arms
Excavator_Param.LinkageArm.Width = 30e-3;

Excavator_Param.Link.LinkageArmRadius = 15e-3;
Excavator_Param.Link.LinkageArmLength = 2*(Excavator_Param.LinkageArm.Width) + (Excavator_Param.Stick.Width);

Excavator_Param.LinkageArmLong.Width = 150e-3;
Excavator_Param.LinkageArmLong.CutoutWidth = 25e-3;


%Bucket
Excavator_Param.Bucket.ThinWidth = 25e-3;


Excavator_Param.Link.BucketTopRadius = 15e-3;
Excavator_Param.Link.BucketTopLength = 2*(Excavator_Param.Bucket.ThinWidth) + (Excavator_Param.Stick.Width);