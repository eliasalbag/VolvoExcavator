Excavator_Param.DirectionalValve.SpoolTravel = 0.005;
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
Excavator_Param.Bom.Length = 1903.392e-3;
Excavator_Param.Bom.Weight = 200;%[kg]
Excavator_Param.Bom.InitAngle = 473.25; 
%571.8

%Stick
Excavator_Param.Stick.Width = 150e-3;
Excavator_Param.Stick.CutoutWidth = 25e-3;
Excavator_Param.Stick.InnerWidth = Excavator_Param.Stick.Width - 2*Excavator_Param.Stick.CutoutWidth;
Excavator_Param.Stick.Length = 1351.485e-3;
Excavator_Param.Stick.Weight = 150;%[kg]
Excavator_Param.Stick.InitAngle = 373.21;


%Linkage arms
Excavator_Param.LinkageArm.Width = 30e-3;

Excavator_Param.Link.LinkageArmRadius = 15e-3;
Excavator_Param.Link.LinkageArmLength = 2*(Excavator_Param.LinkageArm.Width) + (Excavator_Param.Stick.Width);

Excavator_Param.LinkageArmLong.Width = 150e-3;
Excavator_Param.LinkageArmLong.CutoutWidth = 25e-3;


%Bucket
Excavator_Param.Bucket.ThinWidth = 25e-3;
Excavator_Param.Bucket.Length = 590.267e-3;
Excavator_Param.Link.BucketTopRadius = 15e-3;
Excavator_Param.Link.BucketTopLength = 2*(Excavator_Param.Bucket.ThinWidth) + (Excavator_Param.Stick.Width);
Excavator_Param.Bucket.Weight = 70; %[kg]
Excavator_Param.Bucket.InitAngle = 464.23;




%Joints
Excavator_Param.Joint1.Damping = 100;
Excavator_Param.Joint2.Damping = 100;

%Hydraulics
Excavator_Param.Cylinder1.Damping = 1000;
Excavator_Param.Cylinder1.PistonCrossSectionalAreaChamberA = 0.125;
Excavator_Param.Cylinder1.PistonCrossSectionalAreaChamberB = 0.125;
Excavator_Param.Cylinder1.PistonStroke = 580;
Excavator_Param.Cylinder1.DeadVolumeChamberA = 0.05;
Excavator_Param.Cylinder1.DeadVolumeChamberB = 0.05;

Excavator_Param.Cylinder2.Damping = 1000;
Excavator_Param.Cylinder2.PistonCrossSectionalAreaChamberA = 0.125;
Excavator_Param.Cylinder2.PistonCrossSectionalAreaChamberB = 0.125;
Excavator_Param.Cylinder2.PistonStroke = 580;
Excavator_Param.Cylinder2.DeadVolumeChamberA = 0.05;
Excavator_Param.Cylinder2.DeadVolumeChamberB = 0.05;