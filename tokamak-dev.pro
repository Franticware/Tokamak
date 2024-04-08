 

HEADERS += \
    glapp/tok_sample_glapp.h \
    include/math/ne_debug.h \
    include/math/ne_f32.h \
    include/math/ne_math.h \
    include/math/ne_math_m3_inline.h \
    include/math/ne_math_m4_inline.h \
    include/math/ne_math_q_inline.h \
    include/math/ne_math_t3_inline.h \
    include/math/ne_math_v2_inline.h \
    include/math/ne_math_v3_inline.h \
    include/math/ne_math_v4_inline.h \
    include/math/ne_smath.h \
    include/math/ne_type.h \
    include/tokamak.h \
    tokamaksrc/src/collision.h \
    tokamaksrc/src/collision2.h \
    tokamaksrc/src/constraint.h \
    tokamaksrc/src/containers.h \
    tokamaksrc/src/dcd.h \
    tokamaksrc/src/message.h \
    tokamaksrc/src/rigidbody.h \
    tokamaksrc/src/scenery.h \
    tokamaksrc/src/simulator.h \
    tokamaksrc/src/stack.h

SOURCES += \
    glapp/tok_sample_glapp.cpp \
    tokamaksrc/src/boxcylinder.cpp \
    tokamaksrc/src/collision.cpp \
    tokamaksrc/src/collisionbody.cpp \
    tokamaksrc/src/constraint.cpp \
    tokamaksrc/src/cylinder.cpp \
    tokamaksrc/src/dcd.cpp \
    tokamaksrc/src/lines.cpp \
    tokamaksrc/src/ne_interface.cpp \
    tokamaksrc/src/region.cpp \
    tokamaksrc/src/restcontact.cpp \
    tokamaksrc/src/rigidbody.cpp \
    tokamaksrc/src/rigidbodybase.cpp \
    tokamaksrc/src/scenery.cpp \
    tokamaksrc/src/simulator.cpp \
    tokamaksrc/src/solver.cpp \
    tokamaksrc/src/sphere.cpp \
    tokamaksrc/src/stack.cpp \
    tokamaksrc/src/tricollision.cpp

#SOURCES += glapp/balljoints.cpp
#SOURCES += glapp/breakageobjects.cpp
#SOURCES += glapp/car.cpp
#SOURCES += glapp/hingejointmotoron.cpp
#SOURCES += glapp/hingejoints.cpp
SOURCES += glapp/raddude.cpp
#SOURCES += glapp/rigidparticlesandterrain.cpp
#SOURCES += glapp/sccollision.cpp
#SOURCES += glapp/stackingobjects.cpp
#SOURCES += glapp/tank.cpp

INCLUDEPATH += include tokamaksrc/src
LIBS += -lSDL2
