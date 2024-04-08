#include "tok_sample_glapp.h"

#define TOKAMAK_SAMPLE_NAME "car"
const char* tokamakSampleTitle = TOKAMAK_SAMPLE_TITLE_COMMON TOKAMAK_SAMPLE_NAME;

const int32_t WALL_NUMBER = 1;

#define XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE 7849
#define XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE 8689
#define XINPUT_GAMEPAD_TRIGGER_THRESHOLD 30

neV4 vLightWorld[NUM_LIGHT] = { { 1.f, 2.f, 1.f, 0.f }, { -1.f, 1.f, 1.f, 0.f } };

neV4 vLightColor[NUM_LIGHT] = { { 0.7f, 0.7f, 0.7f, 0.f }, { 0.5f, 0.5f, 0.5f, 0.f } };

/*const neV3 g_MinBound = { -4.0f, -GROUND_Y, -6.0f };
const neV3 g_MaxBound = { 4.0f, GROUND_Y, 6.0f };*/

struct DemoData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
};

DemoData gFloor = { { 0.0f, -11.0f, 0.0f }, { 200.0f, 2.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } };

#define N_BARRIERS 4

DemoData gBarriers[N_BARRIERS] = { { { 0.0f, -11.0f, -100.0f }, { 200.0f, 5.0f, 2.0f }, { 0.3f, 0.3f, 0.6f } },
    { { 0.0f, -11.0f, 100.0f }, { 200.0f, 5.0f, 2.0f }, { 0.3f, 0.3f, 0.6f } },
    { { -100.0f, -11.0f, 0.0f }, { 2.0f, 5.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } },
    { { 100.0f, -11.0f, 0.0f }, { 2.0f, 5.0f, 200.0f }, { 0.3f, 0.3f, 0.6f } }
};

#define N_RAMPS 4

struct RampData
{
    neV3 pos;
    neV3 boxSize;
    neV3 colour;
    neV3 angle;
};

RampData gRamps[N_RAMPS] =
{
    { { -40.0f, -12.0f, -50.0f }, { 30.0f, 2.0f, 30.0f }, { 0.3f, 0.3f, 0.6f }, { 0.0f, 0.0f, NE_PI * 0.07f } },
    { { 50.0f, -12.0f, 30.0f }, { 30.0f, 2.0f, 30.0f }, { 0.3f, 0.3f, 0.6f }, { -NE_PI * 0.07f, 0.0f, 0.0f } },
    { { -30.0f, -12.0f, 50.0f }, { 30.0f, 2.0f, 30.0f }, { 0.3f, 0.3f, 0.6f }, { 0.0f, 0.0f, -NE_PI * 0.07f } },
    { { 50.0f, -12.0f, -30.0f }, { 30.0f, 2.0f, 30.0f }, { 0.3f, 0.3f, 0.6f }, { -NE_PI * 0.07f, 0.0f, 0.0f } }
};

const int32_t N_CARS = 10;
//		TARGET_CAR = 0,
const int32_t N_BODY_BOXES = 2;
const int32_t N_PARTS = 3;
const int32_t N_RENDER_PRIMITIVES = (N_BODY_BOXES + N_PARTS);
const int32_t MAX_OVERLAPPED_PAIR = 1000;
const f32 WHEEL_DIAMETER = 0.9f;
const f32 WHEEL_WIDTH = 0.3f;
const f32 MAX_SPEED = 5.0f;
const f32 MAX_REVERSE_SPEED = -2.0f;
const f32 MAX_STEER = 0.7f;
const f32 MAX_SLIDE = 0.90f;

struct SensorData
{
    neV3 pos;
};

SensorData sensorData[4] =
{
    { 2.0f, 0.0f, 1.5f },
    { 2.0f, 0.0f, -1.5f },
    { -2.0f, 0.0f, 1.5f },
    { -2.0f, 0.0f, -1.5f },
};

class CControllerCB : public neRigidBodyControllerCallback
{
public:
    void RigidBodyControllerCallback(neRigidBodyController* controller, float timeStep);
};

CControllerCB cb;

class CSampleCar;

class CCar
{
public:
    void MakeCar(neSimulator* sim, neV3& pos);

    void MakeParts(neSimulator* sim, neV3& pos);

    void CarController(neRigidBodyController* controller);

    void Process();

    CRenderPrimitive* AddRenderPrimitive()
    {
        return &carRenderPrimitives[nextRenderPrim++];
    }

public:
    CCar()
    {
        id = -1;

        accel = 0.0f;

        steer = 0.0f;

        slide = 0.0f;

        gameWorld = NULL;

        nextRenderPrim = 0;
    }

    neRigidBody* carRigidBody;

    neRigidBody* carParts[N_PARTS];

    CRenderPrimitive carRenderPrimitives[N_RENDER_PRIMITIVES];

    CRenderPrimitive wheelRenderPrimitive;

    neV3 displayLines[4][2];

    f32 suspensionLength[4];

    CSampleCar* gameWorld;

    int32_t id;

    f32 accel;

    f32 steer;

    f32 slide;

    neV3 steerDir;

    int32_t nextRenderPrim;
};

class CSampleCar
{
public:
    neSimulator* sim;
    neAllocatorDefault allocator;
    nePerformanceReport perfReport;

    CRenderPrimitive groundRender;
    neAnimatedBody* ground;

    CRenderPrimitive barrierRenders[N_BARRIERS];
    neAnimatedBody* barriers[N_BARRIERS];

    CRenderPrimitive rampRenders[N_RAMPS];
    neAnimatedBody* ramps[N_RAMPS];

    CCar cars[N_CARS];

public:
    CSampleCar()
        : sim(NULL) {};

    void CreateSimulator();

    void CreateGround();

    void Reset();

    void Cleanup();
};

CSampleCar sample;

void CSampleCar::CreateSimulator()
{
    if (sim)
    {
        neSimulator::DestroySimulator(sim);

        sim = NULL;
    }
    // creat the physics simulation

    neSimulatorSizeInfo sizeInfo;

    sizeInfo.rigidBodiesCount = (N_PARTS + 1) * N_CARS;
    sizeInfo.animatedBodiesCount = WALL_NUMBER + N_RAMPS + N_BARRIERS + 1;
    sizeInfo.geometriesCount = (N_RENDER_PRIMITIVES + 1) * N_CARS + WALL_NUMBER;
    sizeInfo.overlappedPairsCount = MAX_OVERLAPPED_PAIR;
    {
        // dont need any of these
        sizeInfo.rigidParticleCount = 0;
        sizeInfo.terrainNodesStartCount = 200;
    }
    neV3 gravity;
    gravity.Set(0.0f, -8.0f, 0.0f);

    sim = neSimulator::CreateSimulator(sizeInfo, &allocator, &gravity);
}

void CSampleCar::CreateGround()
{
    ground = sim->CreateAnimatedBody();

    neGeometry* geom = ground->AddGeometry();

    geom->SetBoxSize(gFloor.boxSize);

    ground->UpdateBoundingInfo();

    ground->SetPos(gFloor.pos);

    groundRender.SetGraphicBox(gFloor.boxSize[0], gFloor.boxSize[1], gFloor.boxSize[2]);

    for (int32_t i = 0; i < N_BARRIERS; i++)
    {
        barriers[i] = sim->CreateAnimatedBody();

        geom = barriers[i]->AddGeometry();

        geom->SetBoxSize(gBarriers[i].boxSize);

        barriers[i]->UpdateBoundingInfo();

        barriers[i]->SetPos(gBarriers[i].pos);

        barrierRenders[i].SetGraphicBox(gBarriers[i].boxSize[0], gBarriers[i].boxSize[1], gBarriers[i].boxSize[2]);

        barrierRenders[i].SetDiffuseColor(0.6f, 0.8f, 0.5f, 1);
    }

    for (int32_t i = 0; i < N_RAMPS; i++)
    {
        ramps[i] = sim->CreateAnimatedBody();

        geom = ramps[i]->AddGeometry();

        geom->SetBoxSize(gRamps[i].boxSize);

        ramps[i]->UpdateBoundingInfo();

        ramps[i]->SetPos(gRamps[i].pos);

        neM3 rot;
        rot.RotateXYZ(gRamps[i].angle);

        ramps[i]->SetRotation(rot);

        rampRenders[i].SetGraphicBox(gRamps[i].boxSize[0], gRamps[i].boxSize[1], gRamps[i].boxSize[2]);

        rampRenders[i].SetDiffuseColor(0.6f, 0.2f, 0.8f, 1);
    }
}

void CSampleCar::Reset()
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

    neV3 carPos;

    carPos.Set(0.0f, -1.0f, 10.0f);

    for (int i = 0; i < N_CARS; i++)
    {
        cars[i].id = i;

        cars[i].MakeCar(sim, carPos);

        carPos[0] -= 7;
    }
}

void CSampleCar::Cleanup()
{
    for (int32_t i = 0; i < N_CARS; i++)
    {
        for (int32_t j = 0; j < N_RENDER_PRIMITIVES; j++)
        {
            sample.cars[i].nextRenderPrim = 0;
        }
    }

    neSimulator::DestroySimulator(sample.sim);

    sim = NULL;
}

void CCar::MakeCar(neSimulator* sim, neV3& pos)
{
    f32 mass = 1.0f;

    neRigidBody* rigidBody = sim->CreateRigidBody();

    //	rigidBody->SetSleepingParameter(0.01f);

    rigidBody->CollideConnected(true);

    neGeometry* geom = rigidBody->AddGeometry();

    neV3 boxSize;
    {
        // the car consist of two boxes

        boxSize.Set(6.0f, 1.0f, 3.2f);

        geom->SetBoxSize(boxSize);

        neT3 t;

        t.SetIdentity();

        t.pos.Set(0.0f, 0.25f, 0.0f);

        geom->SetTransform(t);

        geom = rigidBody->AddGeometry();

        boxSize.Set(2.5f, 1.0f, 2.8f);

        geom->SetBoxSize(boxSize[0], boxSize[1], boxSize[2]);

        t.pos.Set(-0.6f, 1.0f, 0.0f);

        geom->SetTransform(t);
    }
    {
        // add 4 sensors to the rigid body
        neSensor* sn;

        for (int32_t si = 0; si < 4; si++)
        {
            sn = rigidBody->AddSensor();

            neV3 snPos, snDir;

            snDir.Set(0.0f, -1.0f, 0.0f);

            snPos.Set(sensorData[si].pos);

            sn->SetLineSensor(snPos, snDir);
        }

        // add a controller to the rigid body

        neRigidBodyController* controller;

        controller = rigidBody->AddController(&cb, 0);
        (void)controller;
    }

    neV3 tensorSize;

    tensorSize = boxSize;

    rigidBody->UpdateBoundingInfo();

    rigidBody->SetInertiaTensor(neBoxInertiaTensor(tensorSize, mass));

    rigidBody->SetMass(mass);

    rigidBody->SetUserData(this);

    carRigidBody = rigidBody;

    {
        // setup the display boxes for the car

        rigidBody->BeginIterateGeometry();

        while ((geom = rigidBody->GetNextGeometry()))
        {
            // neSimpleToken * token = gw.MakeToken(geom);

            // token->SetDiffuseColour(neV4(0.2f, 0.2f, 0.8f, 1.0f));

            // token->SetAmbientColour(neV4(0.0f, 0.2f, 0.3f, 1.0f));

            neV3 boxSize;

            geom->GetBoxSize(boxSize);

            CRenderPrimitive* renderPrim = AddRenderPrimitive();

            renderPrim->SetGraphicBox(boxSize[0], boxSize[1], boxSize[2]);

            geom->SetUserData(renderPrim);

            renderPrim->SetDiffuseColor(0.2f, 0.2f, 0.8f, 1);
        }
    }

    wheelRenderPrimitive.SetGraphicCylinder(WHEEL_DIAMETER * 0.5f, WHEEL_WIDTH);

    // set the car position

    carRigidBody->SetPos(pos);

    // add bonnet to the car

    MakeParts(sim, pos);
}

struct PartData
{
    neV3 boxSize;
    neV3 position;
    neV3 jointPos;
    neV3 jointRot;
    f32 lowerLimit;
    f32 upperLimit;
};

PartData parts[] =
{
    {
        // Bonet
        { 2.0f, 0.1f, 3.0f },
        { 1.65f, 0.8f, 0.0f },
        { 0.65f, 0.8f, 0.0f },
        { NE_PI * 0.5f, 0.0f, 0.0f },
        0.0f,
        NE_PI * 0.4f,
    },
    {
        // Left Door
        { 2.0f, 0.8f, 0.1f },
        { -0.2f, 0.25f, 1.6f },
        { 0.65f, 0.25f, 1.6f },
        { 0.0f, 0.0f, 0.0f },
        0.0f,
        NE_PI * 0.4f,
    },
    {
        // Right Door
        { 2.0f, 0.8f, 0.1f },
        { -0.2f, 0.25f, -1.6f },
        { 0.65f, 0.25f, -1.6f },
        { 0.0f, 0.0f, 0.0f },
        -NE_PI * 0.4f,
        0.0f,
    },
};

void CCar::MakeParts(neSimulator* sim, neV3& pos)
{
    int32_t i;

    for (i = 0; i < N_PARTS; i++)
    {
        neRigidBody* rb = sim->CreateRigidBody();

        if (id == 0) // make it harder to sleep
        {
            rb->SetSleepingParameter(0.1f);
        }

        rb->SetPos(parts[i].position + pos);

        neGeometry* geom = rb->AddGeometry();

        geom->SetBoxSize(parts[i].boxSize);

        rb->SetMass(0.01f);

        rb->UpdateBoundingInfo();

        rb->SetInertiaTensor(neBoxInertiaTensor(parts[i].boxSize, 0.01f));

        neJoint* joint = sim->CreateJoint(rb, carRigidBody);

        neT3 trans;

        trans.pos = parts[i].jointPos + pos;

        trans.rot.RotateXYZ(parts[i].jointRot);

        joint->SetJointFrameWorld(trans);

        joint->SetType(neJoint::NE_JOINT_HINGE);

        joint->SetLowerLimit(parts[i].lowerLimit);

        joint->SetUpperLimit(parts[i].upperLimit);

        joint->SetEpsilon(0.0f);

        joint->SetIteration(4);

        joint->EnableLimit(true);

        joint->Enable(true);

        carParts[i] = rb;

        CRenderPrimitive* renderPrim = AddRenderPrimitive();

        renderPrim->SetGraphicBox(parts[i].boxSize[0], parts[i].boxSize[1], parts[i].boxSize[2]);

        geom->SetUserData(renderPrim);
    }
}

void MyAppInit()
{
    // Setup the camera
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
    f32 t = 1.0f / 30.0f; //(f32)delta / 1000.0f;

    // sim->Advance(TIME_STEP, 1, &g_PerfReport);
    sample.sim->Advance(t, 1.0f / 30.0f, 1.0f / 30.0f, NULL);

    neV3 vecEye;
    vecEye.Set(30.0f, 12.0f, 40.0f);

    neV3 v = sample.cars[0].carRigidBody->GetPos();
    neV3 vecAt;
    vecAt.Set(v[0], v[1], v[2]);

    neV3 offset;
    offset.Set(0.0f, 0.0f, 80.0f);

    vecEye = vecAt + offset;

    vecEye[1] = 40.0f;

    g_Camera.SetViewParams(vecEye, vecAt);

    {
        // Controller is connected
        sample.cars[0].Process();
    }
}

void OnMyAppFrameRender()
{
    neT3 t;

    t = sample.ground->GetTransform();

    t.MakeD3DCompatibleMatrix();

    sample.groundRender.Render(&t);

    for (int32_t i = 0; i < N_BARRIERS; i++)
    {
        t = sample.barriers[i]->GetTransform();

        t.MakeD3DCompatibleMatrix();

        sample.barrierRenders[i].Render(&t);
    }

    for (int32_t i = 0; i < N_RAMPS; i++)
    {
        t = sample.ramps[i]->GetTransform();

        t.MakeD3DCompatibleMatrix();

        sample.rampRenders[i].Render(&t);
    }

    for (int32_t i = 0; i < N_CARS; i++)
    {
        CCar* car = &sample.cars[i];

        t = car->carRigidBody->GetTransform();

        car->carRigidBody->BeginIterateGeometry();

        while (neGeometry* geom = car->carRigidBody->GetNextGeometry())
        {
            neT3 t2 = geom->GetTransform();

            neT3 t3 = t * t2;

            t3.MakeD3DCompatibleMatrix();

            CRenderPrimitive* rp = (CRenderPrimitive*)geom->GetUserData().p;

            rp->Render(&t3);
        }

        for (int32_t j = 0; j < N_PARTS; j++)
        {
            t = car->carParts[j]->GetTransform();

            t.MakeD3DCompatibleMatrix();

            car->carParts[j]->BeginIterateGeometry();

            neGeometry* geom = car->carParts[j]->GetNextGeometry();

            CRenderPrimitive* rp = (CRenderPrimitive*)geom->GetUserData().p;

            rp->Render(&t);
        }

        for (int32_t j = 0; j < 4; j++)
        {
            // GETDISPLAY->DrawLine(2, colour, &displayLines[i][0]);

            neT3 wheel2World, suspension2Body, tire2Suspension, cylinder2Tire;

            suspension2Body.SetIdentity();

            suspension2Body.pos.Set(0.0f, -car->suspensionLength[j] + WHEEL_DIAMETER / 2.0f, 0.0f);

            suspension2Body.pos += sensorData[j].pos;

            cylinder2Tire.SetIdentity();

            neV3 rot;
            rot.Set(NE_PI * 0.5f, 0.0f, 0.0f);

            cylinder2Tire.rot.RotateXYZ(rot);

            tire2Suspension.SetIdentity();

            if (j == 0 || j == 1)
            {
                rot.Set(0.0f, car->steer, 0.0f);

                tire2Suspension.rot.RotateXYZ(rot);
            }

            wheel2World = car->carRigidBody->GetTransform() * suspension2Body * tire2Suspension * cylinder2Tire;

            if (j == 0 || j == 1)
            {
                car->steerDir = wheel2World.rot[0];
            }
            wheel2World.MakeD3DCompatibleMatrix();

            car->wheelRenderPrimitive.Render(&wheel2World);
        }
    }

    const char* str[6];

    str[0] = "Tokamak Car Sample";
    str[1] = "Controls:";
    str[2] = "'S' -> accelerate";
    str[3] = "'X' -> brake";
    str[4] = "'K', 'L' -> turn";
    str[5] = "";

    MyRenderText(str, 6);
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
        } break;

        default:
            break;
        }
    }
}

void CControllerCB::RigidBodyControllerCallback(neRigidBodyController* controller, float timeStep)
{
    (void)timeStep;
    neRigidBody* rb = controller->GetRigidBody();

    CCar* car = (CCar*)(rb->GetUserData().p);

    car->CarController(controller);
}

void CCar::CarController(neRigidBodyController* controller)
{
    neT3 body2World = carRigidBody->GetTransform();

    carRigidBody->BeginIterateSensor();

    neSensor* sn;

    neV3 force, torque;

    force.SetZero();

    torque.SetZero();

    int32_t i = 0;

    while ((sn = carRigidBody->GetNextSensor()))
    {
        f32 k = 15.0f; // spring constant

        f32 u = 3.f; // damping constant

        if (i == 2 || i == 3)
        {
            k = 10.0f;
        }

        // add spring force

        f32 depth = sn->GetDetectDepth();

        // gw.car.suspensionLength[i] = 1.0f - depth;
        suspensionLength[i] = 1.0f - depth;

        if (depth == 0.0f)
        {
            i++;
            continue;
        }
        // if (depth > 0.7f)
        //	depth = 0.7f;

        neV3 groundNormal = sn->GetDetectNormal();

        neV3 lineNormal = body2World.rot * sn->GetLineUnitVector();

        neV3 linePos = body2World * sn->GetLinePos();

        // f32 dot = lineNormal.Dot(groundNormal) * -1.0f;

        // if (dot <= 0.7f)
        //	continue;

        neV3 f = depth * lineNormal * -k; // * dot;

        force += f;

        neV3 r = linePos - carRigidBody->GetPos();

        torque += r.Cross(f);

        // add damping force

        f32 speed = carRigidBody->GetVelocityAtPoint(r).Dot(lineNormal);

        f = -speed * lineNormal * u;

        force += f;

        torque += r.Cross(f);

        // add friction force

        neV3 vel = carRigidBody->GetVelocityAtPoint(sn->GetDetectContactPoint() - carRigidBody->GetPos());

        vel.RemoveComponent(groundNormal);

        if (i == 0 || i == 1)
        {
            // steering

            vel.RemoveComponent(steerDir);
        }
        else
        {
            vel.RemoveComponent(body2World.rot[0]); // rear wheel always parallel to car body
        }

        f = vel;

        f.Normalize();

        f *= -7.0f;

        if (i == 2 || i == 3)
        {
            f *= (1.0f - slide);
            if (slide > MAX_SLIDE)
            {
                f *= 0.02;
            }
        }
        f[1] = 0.0f;

        force += f;

        r = sn->GetDetectContactPoint() - carRigidBody->GetPos();

        torque += r.Cross(f);

        // driving force

        if (i == 2 || i == 3) // rear wheel
        {
            f = body2World.rot[0];

            f *= accel;

            force += f;

            torque += r.Cross(f);
        }
        /*
                        gw.car.displayLines[i][0] = linePos;

                        gw.car.displayLines[i][1] = sn->GetDetectContactPoint();
        */
        i++;
    }

    // drag
    f32 dragConstant = 0.3f;

    neV3 vel = carRigidBody->GetVelocity();

    f32 dot = vel.Dot(body2World.rot[0]);

    neV3 drag = dot * body2World.rot[0] * -dragConstant;

    force += drag;

    controller->SetControllerForce(force);

    controller->SetControllerTorque(torque);
}

void CCar::Process()
{
    if (sdlGetAsyncKeyState(SDLK_s))
    {
        accel += 0.1f;

        if (accel > MAX_SPEED)
        {
            accel = MAX_SPEED;
        }
    }
    else if (sdlGetAsyncKeyState(SDLK_x))
    {
        accel -= 0.1f;

        if (accel < MAX_REVERSE_SPEED)
        {
            accel = MAX_REVERSE_SPEED;
        }
    }
    else
    {
        accel *= 0.999999f;
    }

    if (sdlGetAsyncKeyState(SDLK_k))
    {
        steer += 0.02f;
        if (steer > MAX_STEER)
        {
            steer = MAX_STEER;

            // slide += 0.05f;
        }
    }
    else if (sdlGetAsyncKeyState(SDLK_l))
    {
        steer -= 0.02f;
        if (steer < -MAX_STEER)
        {
            steer = -MAX_STEER;

            // slide += 0.05f;
        }
    }
    else
    {
        steer *= 0.9f;

        slide *= 0.9f;
    }

    if (slide > MAX_SLIDE)
    {
        slide = MAX_SLIDE;
    }

    if (sdlGetAsyncKeyState(SDLK_t))
    {
        slide = MAX_SLIDE + 0.5;
    }
}
