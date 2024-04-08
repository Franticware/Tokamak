#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "hinge joints"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

// const int32_t MAX_OVERLAPPED_PAIR = 300;
const int32_t WALL_NUMBER = 1;
// const f32 EPSILON  = 0.1f;

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

class CSamplehingeJoints
{
public:
    CSamplehingeJoints()
    {
        paused = false;
    }

    void Initialise();

    void Shutdown();

    void Process();

    void InititialisePhysics();

public:
    enum
    {
        N_BODY = 40,

        N_TOTAL_BODIES = N_BODY,
    };

    neSimulator* sim;

    neRigidBody* rigidBodies[N_TOTAL_BODIES];

    CRenderPrimitive renderPrimitives[N_BODY];

    neJoint* lastJoint;

    neAllocatorDefault all;

    nePerformanceReport perfReport;

    bool paused;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;
};

CSamplehingeJoints sample;

void CSamplehingeJoints::Initialise()
{
    InititialisePhysics();
}

void CSamplehingeJoints::Process()
{
    int32_t actOnBody = N_TOTAL_BODIES - 1;

    if (sdlGetAsyncKeyState(SDLK_t))
    {
        neV3 vel;

        vel.Set(0.0f, 30.0f, 0.0f);

        rigidBodies[actOnBody]->SetVelocity(vel);
    }
}

void CSamplehingeJoints::Shutdown()
{
    neSimulator::DestroySimulator(sim);

    sim = NULL;
}

void CSamplehingeJoints::InititialisePhysics()
{

    neV3 gravity;
    gravity.Set(0.0f, -9.0f, 0.0f);

    f32 linkLength = 0.0f;

    neBool fixedHead = false; // make this true to make chain fix to world
    neBool fixedTail = false;

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = N_TOTAL_BODIES;
    sizeInfo.animatedBodiesCount = WALL_NUMBER;
    sizeInfo.geometriesCount = N_TOTAL_BODIES + WALL_NUMBER;

    int32_t totalBody = N_TOTAL_BODIES + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;

    {
        // dont need any of these
        sizeInfo.terrainNodesStartCount = 0;
        sizeInfo.rigidParticleCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &all, &gravity);

    neRigidBody* lastbox = NULL;

    lastJoint = NULL;

    f32 verticalPos = -5.0f;

    for (int32_t j = 0; j < N_BODY; j++)
    {
        neRigidBody* rigidBody = sim->CreateRigidBody();

        rigidBody->CollideConnected(true);

        neGeometry* geom = rigidBody->AddGeometry();

        neV3 boxSize;

        if (j % 2)
        {
            boxSize.Set(1.0f, 0.5f, 2.0f);
        }
        else
        {
            boxSize.Set(1.0f, 0.5f, 2.2f);
        }

        f32 mass = 0.01f;

        neV3 tensorSize;
        tensorSize = boxSize; //.Set(1.0f, 0.5f, 1.0f);

        geom->SetBoxSize(boxSize[0], boxSize[1], boxSize[2]);

        rigidBody->UpdateBoundingInfo();

        rigidBody->SetInertiaTensor(neBoxInertiaTensor(tensorSize[0], tensorSize[0], tensorSize[0], mass));

        rigidBody->SetMass(mass);

        neV3 pos;

        pos.Set(-0.5f * boxSize[0] - (boxSize[0] + linkLength) * j, verticalPos, 0.0f);

        rigidBody->SetPos(pos);

        rigidBody->SetLinearDamping(0.005f);

        neJoint* joint = NULL;

        neT3 jointFrame;

        jointFrame.rot[0].Set(1.0f, 0.0f, 0.0f);
        jointFrame.rot[1].Set(0.0f, 0.0f, -1.0f);
        jointFrame.rot[2].Set(0.0f, 1.0f, 0.0f);

        if (j != 0)
        {
            joint = sim->CreateJoint(rigidBody, lastbox);

            jointFrame.pos.Set(-(boxSize[0] + linkLength * 0.5f) - (boxSize[0] + linkLength) * (j - 1), verticalPos, 0.0f);

            joint->SetJointFrameWorld(jointFrame);
        }
        else if (fixedHead)
        {
            joint = sim->CreateJoint(rigidBody);

            jointFrame.pos.Set(-(boxSize[0] + linkLength * 0.5f) - (boxSize[0] + linkLength) * (j - 1), verticalPos, 0.0f);

            joint->SetJointFrameWorld(jointFrame);
        }
        if (j == N_BODY - 1)
        {
            lastJoint = joint;
        }
        if (joint)
        {
            joint->SetType(neJoint::NE_JOINT_HINGE);

            joint->SetJointLength(2.0f);

            joint->Enable(true);

            {
                joint->SetLowerLimit(-NE_PI * 0.5f);

                joint->SetUpperLimit(NE_PI * 0.5f);

                joint->EnableLimit(true);

                joint->SetMotor(neJoint::NE_MOTOR_SPEED, 0.0f, 1000.0f);

                // joint->EnableMotor(true);
            }
        }
        if (fixedTail)
        {
            if (j == N_BODY - 1)
            {
                joint = sim->CreateJoint(rigidBody);

                jointFrame.pos.Set(-0.5f * boxSize[0] - (boxSize[0] + linkLength) * j, verticalPos, 0.0f);

                joint->SetJointFrameWorld(jointFrame);

                joint->SetType(neJoint::NE_JOINT_HINGE);

                joint->SetJointLength(2.0f);

                joint->Enable(true);

                {
                    joint->SetLowerLimit(-NE_PI * 0.5f);

                    joint->SetUpperLimit(NE_PI * 0.5f);

                    joint->EnableLimit(true);
                }
            }
        }
        /* set up the graphical models */

        this->renderPrimitives[j].SetGraphicBox(boxSize[0], boxSize[1], boxSize[2]);

        if (j % 2)
        {
            this->renderPrimitives[j].SetDiffuseColor(0.8f, 0.2f, 0.2f, 1.0f);
        }
        else
        {
            this->renderPrimitives[j].SetDiffuseColor(0.2f, 0.2f, 0.8f, 1.0f);
        }

        geom->SetUserData(&renderPrimitives[j]);

        rigidBodies[j] = rigidBody;

        lastbox = rigidBody;
    }
    if (lastJoint)
    {
        lastJoint->SetIteration(4);
    }

    // SetUpRoom

    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(-10.0f, 5.0f, 50.0f);
    neV3 vecAt;
    vecAt.Set(-25.0f, -1.0f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (int32_t i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }

    sample.Initialise();
};

void OnMyAppFrameMove(double fTime, f32 fElapsedTime)
{
    (void)fTime;
    (void)fElapsedTime;
    sample.Process();

    if (!sample.paused)
    {
        sample.sim->Advance(1.0f / 60.0f, 1);
    }
}

void OnMyAppFrameRender()
{
    neT3 t;
    t = sample.ground->GetTransform();
    t.MakeD3DCompatibleMatrix();
    sample.groundRender.Render(&t);

    // Display the boxes

    for (int32_t i = 0; i < sample.N_BODY; i++)
    {
        t = sample.rigidBodies[i]->GetTransform();
        t.MakeD3DCompatibleMatrix();
        sample.renderPrimitives[i].Render(&t);
    }

    const char* str[9];

    str[0] = "Tokamak demo - chain of 40 boxes connected by hinge joints - (c) 2010 Tokamak Ltd";
    str[1] = "Controls:";
    str[2] = "'P' -> pause/unpause the simulation";
    str[3] = "'T' -> lift the end of the chain";
    str[4] = "This sample demonstrate the hinge joints and joint limits.";
    str[5] = "By setting the limit on the joint, each box is constraint to";
    str[6] = "rotate within a specific range. In this case the range is";
    str[7] = "+/- 90 degrees.";
    str[8] = "";

    MyRenderText(str, 9);
}

void OnMyAppDestroyDevice()
{
    sample.Shutdown();
}

void MyAppKeyboardProc(SDL_Keycode nChar, bool bKeyDown, bool bAltDown)
{
    (void)bAltDown;
    if (bKeyDown)
    {
        switch (nChar)
        {
        case SDLK_r:
        {
        } break;

        case SDLK_p:
        {
            sample.paused = !sample.paused;
        }
        break;

        default:
            break;
        }
    }
}
