#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "rad dude"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

const int32_t WALL_NUMBER = 1;

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f },
    { -1.f, 1.f, 1.f, 0.f }
};

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f },
    { 0.5f, 0.5f, 0.5f, 0.f }
};

// const D3DXVECTOR3 g_MinBound( -4.0f, -GROUND_Y, -6.0f );
// const D3DXVECTOR3 g_MaxBound( 4.0f, GROUND_Y, 6.0f );

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

#define RAD_DUDE_BOX 0
#define RAD_DUDE_SPHERE 1
#define RAD_DUDE_CYLINDER 2

struct BoneData
{
    int32_t geometryType;
    float zRotation;
    neV3 pos;
    neV3 size;
    neV3 colour;
};

BoneData bones[] =
{
    { RAD_DUDE_BOX, 0.0f, { 0.0f, 0.0f, 0.0f }, { 0.55f, 0.7f, 0.3f }, { 0.8f, 0.2f, 0.2f } }, // body
    { RAD_DUDE_SPHERE, 0.0f, { 0.0f, 0.55f, 0.0f }, { 0.4f, 0.35f, 0.2f }, { 0.8f, 0.8f, 0.2f } }, // head

    { RAD_DUDE_CYLINDER, -NE_PI / 2.0f, { -0.45f, 0.28f, 0.0f }, { 0.25f, 0.4f, 0.2f }, { 0.2f, 0.2f, 0.8f } }, // right arm
    { RAD_DUDE_CYLINDER, NE_PI / 2.0f, { 0.45f, 0.28f, 0.0f }, { 0.25f, 0.4f, 0.2f }, { 0.2f, 0.2f, 0.8f } }, // left arm

    { RAD_DUDE_BOX, -NE_PI / 2.0f, { -0.9f, 0.28f, 0.0f }, { 0.24f, 0.6f, 0.24f }, { 0.2f, 0.2f, 0.8f } }, // right forearm
    { RAD_DUDE_BOX, NE_PI / 2.0f, { 0.9f, 0.28f, 0.0f }, { 0.24f, 0.6f, 0.24f }, { 0.2f, 0.2f, 0.8f } }, // left forearm

    { RAD_DUDE_CYLINDER, 0.0f, { -0.20f, -0.6f, 0.0f }, { 0.27f, 0.7f, 0.2f }, { 0.3f, 0.7f, 0.2f } }, // right thigh
    { RAD_DUDE_CYLINDER, 0.0f, { 0.20f, -0.6f, 0.0f }, { 0.27f, 0.7f, 0.2f }, { 0.3f, 0.7f, 0.2f } }, // left thigh

    { RAD_DUDE_BOX, 0.0f, { -0.20f, -1.3f, 0.0f }, { 0.3f, 0.8f, 0.3f }, { 0.3f, 0.7f, 0.2f } }, // right leg
    { RAD_DUDE_BOX, 0.0f, { 0.20f, -1.3f, 0.0f }, { 0.3f, 0.8f, 0.3f }, { 0.3f, 0.7f, 0.2f } }, // left leg
};

enum
{
    BONE_BODY,
    BONE_HEAD,
    BONE_RIGHT_ARM,
    BONE_LEFT_ARM,
    BONE_RIGHT_FOREARM,
    BONE_LEFT_FOREARM,
    BONE_RIGHT_THIGH,
    BONE_LEFT_THIGH,
    BONE_RIGHT_LEG,
    BONE_LEFT_LEG,
};

struct JointData
{
    int32_t bodyA;
    int32_t bodyB;
    neV3 pos;
    int32_t type; // 0 = ball joint, 1 = hinge joint
    float xAxisAngle;
    float lowerLimit;
    float upperLimit;
    neBool enableLimit;
    neBool enableTwistLimit;
    float twistLimit;
};

JointData joints[] =
{
    { BONE_HEAD, BONE_BODY, { 0.0f, 0.35f, 0.0f }, 1, 0.0f, -NE_PI / 4.0f, NE_PI / 4.0f, true, false, 0.0f }, // head <-> body

    { BONE_RIGHT_ARM, BONE_BODY, { -0.22f, 0.28f, 0.0f }, 0, NE_PI, 0.0f, NE_PI / 2.5f, true, true, 0.1f }, //
    { BONE_LEFT_ARM, BONE_BODY, { 0.22f, 0.28f, 0.0f }, 0, 0.0f, 0.0f, NE_PI / 2.5f, true, true, 0.1f },

    { BONE_RIGHT_FOREARM, BONE_RIGHT_ARM, { -0.65f, 0.28f, 0.0f }, 1, NE_PI, 0.0f, NE_PI / 2.0f, true, false, 0.f },
    { BONE_LEFT_FOREARM, BONE_LEFT_ARM, { 0.65f, 0.28f, 0.0f }, 1, 0.0f, 0.0f, NE_PI / 2.0f, true, false, 0.f },

    { BONE_RIGHT_THIGH, BONE_BODY, { -0.20f, -0.32f, 0.0f }, 0, NE_PI * 6.0f / 8.0f, 0.0f, NE_PI / 4.0f, true, true, 0.8f },
    { BONE_LEFT_THIGH, BONE_BODY, { 0.20f, -0.32f, 0.0f }, 0, NE_PI * 2.0f / 8.0f, 0.0f, NE_PI / 4.0f, true, true, 0.8f },

    { BONE_RIGHT_LEG, BONE_RIGHT_THIGH, { -0.20f, -0.95f, 0.0f }, 1, -NE_PI * 0.5f, -NE_PI / 2.0f, 0.0f, true, false, 0.f },
    { BONE_LEFT_LEG, BONE_LEFT_THIGH, { 0.20f, -0.95f, 0.0f }, 1, -NE_PI * 0.5f, -NE_PI / 2.0f, 0.0f, true, false, 0.f },
};

const int32_t N_DUDE = 20;
const int32_t BONES_PER_DUDE = 10;
const int32_t JOINTS_PER_DUDE = 9;
const int32_t N_BODY = BONES_PER_DUDE * N_DUDE;
const int32_t N_STEP = 40;

class CSampleRadDude
{
public:
    neSimulator* sim;
    neAllocatorDefault allocator;
    nePerformanceReport perfReport;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;

    neRigidBody* rigidBodies[N_BODY];
    CRenderPrimitive rigidBodiesRender[N_BODY];

    neAnimatedBody* steps[N_STEP];
    CRenderPrimitive stepsRender[N_STEP];

    CSampleRadDude()
        : sim(nullptr) {};

    void CreateSimulator();

    void CreateGround();

    void CreateSteps();

    void CreateRadDude(neV3 position, int32_t& index);

    void Reset();

    void Cleanup();
};

CSampleRadDude sample;

void CSampleRadDude::CreateSimulator()
{
    if (sim)
    {
        neSimulator::DestroySimulator(sim);

        sim = nullptr;
    }
    // creat the physics simulation

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = N_BODY;
    sizeInfo.animatedBodiesCount = WALL_NUMBER + N_STEP;
    sizeInfo.geometriesCount = N_BODY + WALL_NUMBER + N_STEP;
    sizeInfo.constraintsCount = N_BODY * JOINTS_PER_DUDE;
    int32_t totalBody = N_BODY + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = totalBody * (totalBody - 1) / 2;
    neV3 gravity;
    gravity.Set(0.0f, -9.0f, 0.0f);
    {
        // dont need any of these
        sizeInfo.rigidParticleCount = 0;
        // sizeInfo.terrainNodesStartCount = 0;
    }

    sim = neSimulator::CreateSimulator(sizeInfo, &allocator, &gravity);
}

void CSampleRadDude::CreateGround()
{
    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);
}

void CSampleRadDude::CreateSteps()
{
    for (int32_t i = 0; i < N_STEP; i++)
    {
        steps[i] = sim->CreateAnimatedBody();

        neGeometry* geom = steps[i]->AddGeometry();

        const float stepHeight = 0.5f; // 1.2f;

        const float stepDepth = 0.5f; // 1.2f;

        neV3 stepSize;
        stepSize.Set(30.0f, stepHeight, stepDepth);

        geom->SetBoxSize(stepSize);

        steps[i]->UpdateBoundingInfo();

        neV3 pos;

        pos.Set(0.0f, 8.0f - stepHeight * i, (-1.0f + stepDepth * i));

        steps[i]->SetPos(pos);

        stepsRender[i].SetGraphicBox(30.0f, stepHeight, stepDepth);
    }
}

void CSampleRadDude::CreateRadDude(neV3 position, int32_t& index)
{
    //		const float groundLevel = -10.0f;

    int32_t cur = 0;

    float scale = 1.0f;

    for (int32_t i = 0; i < BONES_PER_DUDE; i++)
    {
        cur = index + i;

        rigidBodies[cur] = sim->CreateRigidBody();

        rigidBodies[cur]->CollideConnected(true);

        neV3 inertiaTensor;

        float mass;

        mass = 8.0f;

        if (i == 0)
        {
            mass = 20.0f;
        }
        else if (i == 8 || i == 9)
        {
            mass = 8.0f;
        }

        neGeometry* geom = rigidBodies[cur]->AddGeometry();

        switch (bones[i].geometryType)
        {
        case RAD_DUDE_BOX:

            geom->SetBoxSize(bones[i].size[0] * scale, bones[i].size[1] * scale, bones[i].size[2] * scale);

            inertiaTensor = neBoxInertiaTensor(bones[i].size[0] * scale,
                                               bones[i].size[1] * scale,
                                               bones[i].size[2] * scale, mass);

            rigidBodiesRender[cur].SetGraphicBox(bones[i].size[0] * scale, bones[i].size[1] * scale, bones[i].size[2] * scale);

            break;

        case RAD_DUDE_SPHERE:

            geom->SetSphereDiameter(bones[i].size[0] * scale);

            inertiaTensor = neSphereInertiaTensor(bones[i].size[0] * scale, mass);

            rigidBodiesRender[cur].SetGraphicSphere(bones[i].size[0] * scale * 0.5f);

            break;

        case RAD_DUDE_CYLINDER:

            geom->SetCylinder(bones[i].size[0] * scale, bones[i].size[1] * scale);

            inertiaTensor = neCylinderInertiaTensor(bones[i].size[0], bones[i].size[1], mass);

            rigidBodiesRender[cur].SetGraphicCylinder(bones[i].size[0] * scale * 0.5f, bones[i].size[1] * scale);

            break;
        }

        rigidBodiesRender[cur].SetDiffuseColor(bones[i].colour[0],
                                               bones[i].colour[1],
                                               bones[i].colour[2], 1);

        rigidBodies[cur]->UpdateBoundingInfo();

        rigidBodies[cur]->SetInertiaTensor(inertiaTensor);

        rigidBodies[cur]->SetMass(mass);

        neV3 pos;

        pos = bones[i].pos * scale + position;

        rigidBodies[cur]->SetPos(pos);

        neQ quat;

        neV3 zAxis;
        zAxis.Set(0.0f, 0.0f, 1.0f);

        quat.Set(bones[i].zRotation, zAxis);

        rigidBodies[cur]->SetRotation(quat);

        rigidBodies[cur]->SetSleepingParameter(0.5f); // make it easier to sleep
    }
    neJoint* joint;

    neT3 jointFrame;

    for (int32_t i = 0; i < JOINTS_PER_DUDE; i++)
    {
        joint = sim->CreateJoint(rigidBodies[joints[i].bodyA + index], rigidBodies[joints[i].bodyB + index]);

        jointFrame.SetIdentity();

        jointFrame.pos = joints[i].pos * scale + position;

        if (joints[i].type == 0)
        {
            joint->SetType(neJoint::NE_JOINT_BALLSOCKET);

            neQ q;

            neV3 zAxis;
            zAxis.Set(0.0f, 0.0f, 1.0f);

            q.Set(joints[i].xAxisAngle, zAxis);

            jointFrame.rot = q.BuildMatrix3();
        }
        else
        {
            joint->SetType(neJoint::NE_JOINT_HINGE);

            if (i == 3)
            {
                jointFrame.rot[0].Set(1.0f, 0.0f, 0.0f);
                jointFrame.rot[1].Set(0.0f, 1.0f, 0.0f);
                jointFrame.rot[2].Set(0.0f, 0.0f, 1.0f);
            }
            else if (i == 4)
            {
                jointFrame.rot[0].Set(-1.0f, 0.0f, 0.0f);
                jointFrame.rot[1].Set(0.0f, -1.0f, 0.0f);
                jointFrame.rot[2].Set(0.0f, 0.0f, 1.0f);
            }
            else
            {
                jointFrame.rot[0].Set(0.0f, 0.0f, -1.0f);
                jointFrame.rot[1].Set(-1.0f, 0.0f, 0.0f);
                jointFrame.rot[2].Set(0.0f, 1.0f, 0.0f);
            }
        }

        joint->SetJointFrameWorld(jointFrame);

        if (i == 5 || i == 6) // right
        {
            neT3 body2w = rigidBodies[joints[i].bodyB + index]->GetTransform();

            neT3 w2Body = body2w.FastInverse();

            neM3 m;

            neQ q1, q2;

            neV3 zAxis;
            zAxis.Set(0.0f, 0.0f, 1.0f);

            q1.Set(joints[i].xAxisAngle, zAxis);

            neV3 xAxis;
            xAxis.Set(1.0f, 0.0f, 0.0f);

            q2.Set(-NE_PI * 0.30f, xAxis);

            neQ q;
            q = q2 * q1;

            m = q.BuildMatrix3();

            neT3 frame2body;

            frame2body.rot = w2Body.rot * m;

            frame2body.pos = w2Body * jointFrame.pos;

            // neT3 frame2w = body2w * frame2body;

            joint->SetJointFrameB(frame2body);
        }
        joint->SetLowerLimit(joints[i].lowerLimit);

        joint->SetUpperLimit(joints[i].upperLimit);

        if (joints[i].enableLimit)
        {
            joint->EnableLimit(true);
        }

        if (joints[i].enableTwistLimit)
        {
            joint->SetLowerLimit2(joints[i].twistLimit);

            joint->EnableLimit2(true);
        }

        joint->SetIteration(4);

        joint->Enable(true);
    }
    index = cur + 1;
}

void CSampleRadDude::Reset()
{
    Cleanup();

    // reset the camera position
    neV3 vecEye;
    vecEye.Set(30.0f, 12.0f, 40.0f); //(0.0f, -GROUND_Y + 1.7f, 0.0f);
    neV3 vecAt;
    vecAt.Set(0.0f, 0.0f, 1.0f); //(0.0f, -GROUND_Y + 1.7f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    CreateSimulator();

    CreateGround();

    CreateSteps();

    neV3 position;

    position.SetZero();

    int32_t i = 0;

    for (int32_t j = 0; j < N_DUDE; j++)
    {
        position.Set(-5 + (j % 5) * 2.0f, 11.0f + 4.0f * (j / 5), 0.0f);

        CreateRadDude(position, i);
    }
}

void CSampleRadDude::Cleanup()
{
    neSimulator::DestroySimulator(sample.sim);

    sim = nullptr;
}

void MyAppInit()
{
    neV3 vecEye;
    vecEye.Set(0.0f, 0.0f, -10.0f); //(0.0f, -GROUND_Y + 1.7f, 0.0f);
    neV3 vecAt;
    vecAt.Set(0.0f, 0.0f, 1.0f); //(0.0f, -GROUND_Y + 1.7f, 1.0f);
    g_Camera.SetViewParams(vecEye, vecAt);

    for (int32_t i = 0; i < NUM_LIGHT; i++)
    {
        vLightWorld[i].Normalize();
    }
    sample.Reset();
};

void OnMyAppFrameMove(double fTime, float fElapsedTime)
{
    (void)fTime;
    (void)fElapsedTime;
    float t = 1.0f / 30.0f; //(float)delta / 1000.0f;

    // sim->Advance(TIME_STEP, 1, &g_PerfReport);
    sample.sim->Advance(t, 1.0f / 60.0f, 1.0f / 30.0f, nullptr);
}

void OnMyAppFrameRender()
{
    neT3 t;

    t = sample.ground->GetTransform();

    t.MakeD3DCompatibleMatrix();

    sample.groundRender.Render(&t);

    for (int32_t i = 0; i < N_STEP; i++)
    {
        t = sample.steps[i]->GetTransform();

        t.MakeD3DCompatibleMatrix();

        sample.stepsRender[i].Render(&t);
    }

    for (int32_t i = 0; i < N_BODY; i++)
    {
        t = sample.rigidBodies[i]->GetTransform();

        t.MakeD3DCompatibleMatrix();

        sample.rigidBodiesRender[i].Render(&t);
    }

    const char* str[2];

    str[0] = "Tokamak Rad Dude Sample";
    str[1] = "";

    MyRenderText(str, 2);
}

void OnMyAppDestroyDevice()
{
    sample.Cleanup();
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
            sample.Reset();
        }
        break;

        default:
            break;
        }
    }
}
