#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/Config.hpp>
#include <SFML/Audio.hpp>
#include <vector>
#include <box2d/box2d.h>

#define b2_pi 3.14159265359f
#define ARMWIDTH 0.25f
#define SCALE_FACTOR 30.0f	// 30 pixels = 1 meter, this is defined in order to convert SFML coordinates into Box2d coordinates

struct Armwheels {
	float wheelsDensity, wheelsFriction, wheelsRadius, armLength, armAngle, wheelsTorque, LwheelSpeed, RwheelSpeed, pivotTorque, pivotSpeed;
}; // Structure that contains information about the Armwheels, the unit of measure is meter for lengths and radians for angle

struct ArmwheelsIds {
	b2BodyId arm1Id, arm2Id, wheel1Id, wheel2Id;
	b2ShapeId arm1shapeId, arm2shapeId, wheel1shapeId, wheel2shapeId;
	b2JointId pivotjointId, wheel1jointId, wheel2jointId;

}; // Structure that contains the Box2d IDs of the bodies, shapes and joints of the Armwheels

sf::Vector2f convertToSFML(b2Vec2 b2position, const sf::Vector2u& resolution) {	// Utility function to convert the Box2D coordinates to SFML coordinates
	return sf::Vector2f{
		float(resolution.x) / 2.0f + b2position.x * SCALE_FACTOR,
		float(resolution.y) / 2.0f - b2position.y * SCALE_FACTOR // Invert Y
	};
}

ArmwheelsIds createArmwheels(Armwheels armwheels, b2Vec2 pivotPosition, b2WorldId worldId) {

	ArmwheelsIds Ids; // Structure to store the Box2D IDs

	float initialAngle1_rad = -B2_PI / 4.0f; // -45° for the first arm (right)
	float initialAngle2_rad = -3.0f * B2_PI / 4.0f; // -135° for the second arm (left)

	// Create Arm Bodies
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = pivotPosition; // Both arms pivot around the same point
	bodyDef.enableSleep = false;
	// Set initial angular damping 
	bodyDef.angularDamping = 0.1f;

	// Create first arm body (right) with its initial rotation
	bodyDef.rotation = b2MakeRot(initialAngle1_rad);
	b2BodyId body1 = b2CreateBody(worldId, &bodyDef);
	Ids.arm1Id = body1;

	// Create second arm body (left) with its initial rotation
	bodyDef.rotation = b2MakeRot(initialAngle2_rad);
	b2BodyId body2 = b2CreateBody(worldId, &bodyDef);
	Ids.arm2Id = body2;

	// Define the shape ONCE, relative to the body's local origin.
	// Both arms will use this same local definition.
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 1.0f; // Default arm density
	shapeDef.friction = 0.9f; // Default arm friction
	

	b2Capsule capsule;
	capsule.center1 = { 0.0f, 0.0f };             // Start at the body's origin (pivot)
	capsule.center2 = { armwheels.armLength, 0.0f }; // Extend along the local +X axis
	capsule.radius = ARMWIDTH;                   

	// Create and attach the shape to the first arm
	b2ShapeId arm1shape = b2CreateCapsuleShape(body1, &shapeDef, &capsule);
	Ids.arm1shapeId = arm1shape;

	// Create and attach the *same* shape definition to the second arm
	b2ShapeId arm2shape = b2CreateCapsuleShape(body2, &shapeDef, &capsule);
	Ids.arm2shapeId = arm2shape;


	// Create Wheel Bodies 
	// Calculate initial world positions for the wheels based on arm positions and initial angles
	b2Vec2 wheel1EndPos = { armwheels.armLength * std::cos(initialAngle1_rad),
							armwheels.armLength * std::sin(initialAngle1_rad) };
	b2Vec2 wheel1Pos = { pivotPosition.x + wheel1EndPos.x, pivotPosition.y + wheel1EndPos.y };

	b2Vec2 wheel2EndPos = { armwheels.armLength * std::cos(initialAngle2_rad),
							armwheels.armLength * std::sin(initialAngle2_rad) };
	b2Vec2 wheel2Pos = { pivotPosition.x + wheel2EndPos.x, pivotPosition.y + wheel2EndPos.y };


	bodyDef.position = wheel1Pos;
	bodyDef.rotation = b2MakeRot(0.0f); // Wheels start with no rotation relative to world
	b2BodyId body3 = b2CreateBody(worldId, &bodyDef);
	Ids.wheel1Id = body3;

	bodyDef.position = wheel2Pos;
	// bodyDef.rotation is already {0,0}
	b2BodyId body4 = b2CreateBody(worldId, &bodyDef);
	Ids.wheel2Id = body4;


	// Define Wheel Shape (Circle) 
	shapeDef.density = armwheels.wheelsDensity;   
	shapeDef.friction = armwheels.wheelsFriction; 

	b2Circle circle;
	circle.center = { 0.0f, 0.0f }; // Center of the circle is at the body's origin
	circle.radius = armwheels.wheelsRadius;

	// Create and attach the circle shape to the first wheel
	b2ShapeId wheel1shape = b2CreateCircleShape(body3, &shapeDef, &circle);
	Ids.wheel1shapeId = wheel1shape;

	// Create and attach the circle shape to the second wheel
	b2ShapeId wheel2shape = b2CreateCircleShape(body4, &shapeDef, &circle);
	Ids.wheel2shapeId = wheel2shape;


	// Create Joints

	// Pivot Joint (connecting the two arms at the pivot point)
	b2RevoluteJointDef pivotJointDef = b2DefaultRevoluteJointDef();
	pivotJointDef.bodyIdA = body1;
	pivotJointDef.bodyIdB = body2;
	pivotJointDef.localAnchorA = { 0.0f, 0.0f }; 
	pivotJointDef.localAnchorB = { 0.0f, 0.0f }; 
	pivotJointDef.enableMotor = true;          // Enable the motor
	pivotJointDef.maxMotorTorque = armwheels.pivotTorque; // Set max torque
	pivotJointDef.motorSpeed = armwheels.pivotSpeed;      // Initial motor speed 
	pivotJointDef.enableLimit = false;         // Disable angle limits 
	pivotJointDef.enableSpring = false;

	
	b2JointId pivotJoint = b2CreateRevoluteJoint(worldId, &pivotJointDef);
	b2Joint_SetCollideConnected(pivotJoint, false); // Prevent arms from colliding with each other
	Ids.pivotjointId = pivotJoint;


	// Wheel 1 Joint (connecting arm 1 to wheel 1)
	b2RevoluteJointDef wheelJointDef = b2DefaultRevoluteJointDef(); 
	wheelJointDef.bodyIdA = body1; 
	wheelJointDef.bodyIdB = body3; 
	wheelJointDef.localAnchorA = { armwheels.armLength, 0.0f };
	wheelJointDef.localAnchorB = { 0.0f, 0.0f };
	wheelJointDef.enableMotor = true;
	wheelJointDef.maxMotorTorque = armwheels.wheelsTorque;
	wheelJointDef.motorSpeed = armwheels.RwheelSpeed; // Initial speed for right wheel motor
	wheelJointDef.enableLimit = false;

	b2JointId wheel1Joint = b2CreateRevoluteJoint(worldId, &wheelJointDef);
	b2Joint_SetCollideConnected(wheel1Joint, false); // Prevent arm 1 colliding with wheel 1
	Ids.wheel1jointId = wheel1Joint;


	// Wheel 2 Joint (connecting arm 2 to wheel 2)
	// Reuse wheelJointDef, just change bodies and speed if needed
	wheelJointDef.bodyIdA = body2; 
	wheelJointDef.bodyIdB = body4; 
	
	wheelJointDef.localAnchorA = { armwheels.armLength, 0.0f };
	
	wheelJointDef.localAnchorB = { 0.0f, 0.0f };
	wheelJointDef.motorSpeed = armwheels.LwheelSpeed; // Initial speed for left wheel motor

	b2JointId wheel2Joint = b2CreateRevoluteJoint(worldId, &wheelJointDef);
	b2Joint_SetCollideConnected(wheel2Joint, false); // Prevent arm colliding with wheel
	Ids.wheel2jointId = wheel2Joint;


	std::cout << ("Armwheels successfully created with standard definitions!\n");
	return Ids; // Return the structure containing all the IDs
}

sf::RectangleShape printArms(b2BodyId armId, Armwheels armwheels, b2WorldId worldId, sf::Vector2u resolution) {

	b2Vec2 b2position = b2Body_GetPosition(armId);
	b2Rot b2rotation = b2Body_GetRotation(armId);

	float rotation = b2Rot_GetAngle(b2rotation);
	sf::Angle angle = sf::radians(rotation);
	sf::Vector2f position = convertToSFML(b2position, resolution);
	sf::RectangleShape arm({ armwheels.armLength * SCALE_FACTOR , ARMWIDTH * SCALE_FACTOR });
	arm.setOrigin({ 0.0f , ARMWIDTH * SCALE_FACTOR / 2 });
	arm.setPosition(position);
	arm.setRotation(-angle);
	arm.setFillColor(sf::Color::White);
	return arm;

}
sf::CircleShape printWheels(b2BodyId wheelId, Armwheels armwheels, b2WorldId worldId, sf::Vector2u resolution) {

	b2Vec2 b2position = b2Body_GetPosition(wheelId);
	b2Rot b2rotation = b2Body_GetRotation(wheelId);

	float rotation = b2Rot_GetAngle(b2rotation);
	sf::Angle angle = sf::radians(rotation);
	sf::Vector2f position = convertToSFML(b2position, resolution);
	sf::CircleShape wheel(armwheels.wheelsRadius * SCALE_FACTOR);
	wheel.setOrigin({ armwheels.wheelsRadius * SCALE_FACTOR, armwheels.wheelsRadius * SCALE_FACTOR});
	wheel.setPosition(position);
	wheel.setRotation(angle);
	return wheel;

}
sf::CircleShape printJoints(b2BodyId Id, sf::Vector2u resolution) {

	b2Vec2 b2position = b2Body_GetPosition(Id);
	sf::Vector2f pivotPosition = convertToSFML(b2position, resolution);
	sf::CircleShape pivot((ARMWIDTH / 4) * SCALE_FACTOR);
	pivot.setOrigin({ (ARMWIDTH / 4) * SCALE_FACTOR, (ARMWIDTH / 4) * SCALE_FACTOR });
	pivot.setPosition(pivotPosition);
	pivot.setFillColor(sf::Color::Black);
	return pivot;

}
void printArmwheels(ArmwheelsIds Ids, Armwheels armwheels, b2WorldId worldId, sf::RenderWindow &window, sf::Vector2u resolution) {

	// Need to clear the window first so the frames don't overlap
	window.clear();
	// SFML renderer has origin in the top left corner, with y-axis pointing down, while Box2d has the origin placed in the middle 
	// Conversion is SFML given by the function convertToSFML
	
	// As earlier print the arms first
	sf::RectangleShape arm1 = printArms(Ids.arm1Id, armwheels, worldId, resolution);
	sf::RectangleShape arm2 = printArms(Ids.arm2Id, armwheels, worldId, resolution);

	// Then wheels
	sf::CircleShape wheel1 = printWheels(Ids.wheel1Id, armwheels, worldId, resolution);
	wheel1.setFillColor(sf::Color::Red);	// Red color for the right wheel
	sf::CircleShape wheel2 = printWheels(Ids.wheel2Id, armwheels, worldId, resolution);
	wheel2.setFillColor(sf::Color::Blue);	// Blue color for the left wheel

	// Now the pivots (black dots to representing the joints)
	sf::CircleShape pivot1 = printJoints(Ids.arm1Id, resolution);
	sf::CircleShape pivot2 = printJoints(Ids.wheel1Id, resolution);
	sf::CircleShape pivot3 = printJoints(Ids.wheel2Id, resolution);

	b2Vec2 b2position = b2Body_GetPosition(Ids.arm1Id);
	sf::Vector2f position = convertToSFML(b2position, resolution);;
	sf::CircleShape complete(ARMWIDTH / 2 * SCALE_FACTOR);
	complete.setOrigin({ (ARMWIDTH / 2) * SCALE_FACTOR, (ARMWIDTH / 2) * SCALE_FACTOR });
	complete.setPosition(position);
	complete.setFillColor(sf::Color::White);
	
	window.draw(arm1);
	window.draw(arm2);
	window.draw(complete);
	window.draw(wheel1);
	window.draw(wheel2);
	window.draw(pivot1);
	window.draw(pivot2);
	window.draw(pivot3);

}

void printDebug(ArmwheelsIds Ids) {
	b2Vec2 wheel1Pos = b2Body_GetPosition(Ids.wheel1Id);
	b2Vec2 wheel2Pos = b2Body_GetPosition(Ids.wheel2Id);

	std::cout << "Wheel 1: (" << wheel1Pos.x << ", " << wheel1Pos.y << ")  ";
	std::cout << "Wheel 2: (" << wheel2Pos.x << ", " << wheel2Pos.y << ")";
	float dx = wheel2Pos.x - wheel1Pos.x;
	float dy = wheel2Pos.y - wheel1Pos.y;
	float distance = std::sqrt(dx * dx + dy * dy);
	std::cout << "Distance between wheels: " << distance;
	std::cout << " Joint angle: " << b2RevoluteJoint_GetAngle(Ids.pivotjointId) * (180.0f / b2_pi) << std::endl;
}

void applyDefaultView(sf::Vector2u resolution, sf::RenderWindow& window) {
	sf::View view;
	sf::Vector2f center(float(resolution.x) / 2.f, float(resolution.y) / 2.f);
	sf::Vector2f size(float(resolution.x), float(resolution.y));
	view.setCenter(center);
	view.setSize(size);
	window.setView(view);
}

void adjustView(ArmwheelsIds Ids, sf::Vector2u resolution, sf::RenderWindow &window) {
	sf::Vector2f pivotView = convertToSFML(b2Body_GetPosition(Ids.arm1Id), resolution);	// Get the coordinates of the pivot point
	sf::View currentView = window.getView();	// Get the current view

	// Calculate the borders of the view
	sf::Vector2f viewCenter = currentView.getCenter();
	sf::Vector2f viewSize = currentView.getSize();

	float viewLeft = viewCenter.x - viewSize.x / 2.0f;
	float viewRight = viewCenter.x + viewSize.x / 2.0f;
	float viewTop = viewCenter.y - viewSize.y / 2.0f;
	float viewBottom = viewCenter.y + viewSize.y / 2.0f;

	// Check if the pivot is inside the borders, otherwise, move the view accordingly
	// REMEMBER: SFML coordinates have the y-axis pointing downwards, so the logic for checking the position and moving the view is inverted
	if (pivotView.x > viewRight) {
		currentView.move({ float(resolution.x), 0.0f });
	}
	else if (pivotView.x < viewLeft) {
		currentView.move({ -float(resolution.x), 0.0f });
	}

	if (pivotView.y < viewTop) {
		currentView.move({ 0.0f, -float(resolution.y) });
	}
	else if (pivotView.y > viewBottom) {
		currentView.move({ 0.0f, float(resolution.y) });
	}
	window.setView(currentView);
}

int main() {

	// Generate the world
	sf::Vector2u resolution = { 800, 600 };	// Set the default Resolution for the SFML window
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = { 0.0f, -10.0f };
	b2WorldId worldId = b2CreateWorld(&worldDef);
	// Generate the structure representing the armwheels, change the values below to get different behaviours
	Armwheels armwheels;
	armwheels.armAngle = b2_pi / 2;	// Initialize the angle as 90°
	armwheels.wheelsDensity = 1.0f;
	armwheels.wheelsFriction = 2.0f;
	armwheels.wheelsRadius = 1.0f;
	armwheels.armLength = 4.0f;
	armwheels.pivotSpeed = 2.0f;	// Change these values to affect the speed of the wheels and the speed of the revolute joint
	armwheels.LwheelSpeed = 7.0f;
	armwheels.RwheelSpeed = 7.0f;
	armwheels.pivotTorque = 200.0f;
	armwheels.wheelsTorque = 100.0f;

	b2Vec2 pivotPosition = { 0.0f, 5.0f };

	ArmwheelsIds Ids = createArmwheels(armwheels, pivotPosition, worldId);

	

	float dt = 1.0f / 60.0f;	// Timestep = 60mS, Frequency = 60Hz
	int freq = 60;
	int substep = 4;

	
	sf::ContextSettings settings;
	settings.antiAliasingLevel = 8;

	sf::RenderWindow window(sf::VideoMode(resolution), "Armwheels", sf::Style::Default, sf::State::Windowed, settings);	// Render the window
	window.setFramerateLimit(freq);	// Set the refresh rate of the window
	window.clear();


	// WORLD GENERATION AND PRINT
	// 
	// Generate and print Ground Box
	b2BodyDef groundBodyDef = b2DefaultBodyDef();
	groundBodyDef.position = { 0.0f, -10.0f };
	groundBodyDef.type = b2_staticBody;
	b2BodyId groundId = b2CreateBody(worldId, &groundBodyDef);
	b2Polygon groundBox = b2MakeBox(50.0f, 10.0f);

	b2ShapeDef groundShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);

	sf::RectangleShape ground({ 100.0f * SCALE_FACTOR, 20.0f * SCALE_FACTOR });
	ground.setFillColor(sf::Color::White);
	ground.setOrigin({ (100.0f * SCALE_FACTOR) / 2.0f, (20.0f * SCALE_FACTOR) / 2.0f });

	ground.setPosition({ float(resolution.x) / 2, float(resolution.y) / 2 + 10.0f * SCALE_FACTOR });

	// BOX 
	b2BodyDef boxBodyDef = b2DefaultBodyDef();
	boxBodyDef.position = { 40.0f, 2.5f };
	boxBodyDef.type = b2_staticBody;
	b2BodyId boxId = b2CreateBody(worldId, &boxBodyDef);
	b2Polygon boxBox = b2MakeBox(10.0f, 2.5f);

	b2ShapeDef boxShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(boxId, &boxShapeDef, &boxBox);

	sf::RectangleShape box({ 20.0f * SCALE_FACTOR, 5.0f * SCALE_FACTOR });
	box.setFillColor(sf::Color::White);
	box.setOrigin({ (20.0f * SCALE_FACTOR) / 2.0f, (5.0f * SCALE_FACTOR) / 2.0f });
	box.setPosition({ float(resolution.x) / 2 + 40.0f * SCALE_FACTOR, float(resolution.y) / 2 - 2.5f * SCALE_FACTOR});

	// WALL
	b2BodyDef wallBodyDef = b2DefaultBodyDef();
	wallBodyDef.position = { -75.0f, 30.0f };
	wallBodyDef.type = b2_staticBody;
	b2BodyId wallId = b2CreateBody(worldId, &wallBodyDef);
	b2Polygon wallBox = b2MakeBox(25.0f, 50.0f);

	b2ShapeDef wallShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(wallId, &wallShapeDef, &wallBox);

	sf::RectangleShape wall({ 50.0f * SCALE_FACTOR, 100.0f * SCALE_FACTOR });
	wall.setFillColor(sf::Color::White);
	wall.setOrigin({ (50.0f * SCALE_FACTOR) / 2.0f, (100.0f * SCALE_FACTOR) / 2.0f });
	wall.setPosition({ float(resolution.x) / 2 - 75.0f * SCALE_FACTOR, float(resolution.y) / 2 - 30.0f * SCALE_FACTOR});

	// SECOND GROUND BOX WITH A JUMP
	b2BodyDef ground2BodyDef = b2DefaultBodyDef();
	ground2BodyDef.position = { 110.0f, -10.0f };
	ground2BodyDef.type = b2_staticBody;
	b2BodyId ground2Id = b2CreateBody(worldId, &ground2BodyDef);
	b2Polygon ground2Box = b2MakeBox(50.0f, 10.0f);

	b2ShapeDef ground2ShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(ground2Id, &ground2ShapeDef, &ground2Box);

	sf::RectangleShape ground2({ 100.0f * SCALE_FACTOR, 20.0f * SCALE_FACTOR });
	ground2.setFillColor(sf::Color::White);
	ground2.setOrigin({ (100.0f * SCALE_FACTOR) / 2.0f, (20.0f * SCALE_FACTOR) / 2.0f });

	ground2.setPosition({ float(resolution.x) / 2 + 110.f * SCALE_FACTOR, float(resolution.y) / 2 + 10.0f * SCALE_FACTOR });
	// SPIKES ON THE GROUND
	// Spike1
	b2Vec2 p1 = { 0.0f, 0.0f };     // bottom-left
	b2Vec2 p2 = { 2.0f, 0.0f };     // bottom-right
	b2Vec2 p3 = { 1.0f, 2.0f };     // top

	b2Vec2 vertices[3] = { p1, p2, p3 };
	b2Hull spikeHull = b2ComputeHull(vertices, 3);

	b2Polygon spike = b2MakePolygon(&spikeHull, 0.0f);  

	b2BodyDef spikeBodyDef = b2DefaultBodyDef();
	spikeBodyDef.type = b2_staticBody;
	spikeBodyDef.position = { 77.0f, 0.0f };  
	b2BodyId spikeBodyId = b2CreateBody(worldId, &spikeBodyDef);

	b2ShapeDef spikeShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(spikeBodyId, &spikeShapeDef, &spike);

	sf::ConvexShape spike1;
	spike1.setPointCount(3);
	spike1.setFillColor(sf::Color::White);

	spike1.setPoint(0, { p1.x * SCALE_FACTOR, -p1.y * SCALE_FACTOR });
	spike1.setPoint(1, { p2.x * SCALE_FACTOR, -p2.y * SCALE_FACTOR });
	spike1.setPoint(2, { p3.x * SCALE_FACTOR, -p3.y * SCALE_FACTOR });

	spike1.setPosition({convertToSFML({ 77.0f, 0.0f }, resolution)});

	// Spike2
	p3 = { 1.0f, 4.0f };     

	vertices[2] = p3;
	spikeHull = b2ComputeHull(vertices, 3);

	spike = b2MakePolygon(&spikeHull, 0.0f);  

	spikeBodyDef.position = { 79.0f, 0.0f };  
	b2BodyId spike2BodyId = b2CreateBody(worldId, &spikeBodyDef);

	spikeShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(spike2BodyId, &spikeShapeDef, &spike);

	sf::ConvexShape spike2;
	spike2.setPointCount(3);
	spike2.setFillColor(sf::Color::White);

	spike2.setPoint(0, { p1.x * SCALE_FACTOR, -p1.y * SCALE_FACTOR });
	spike2.setPoint(1, { p2.x * SCALE_FACTOR, -p2.y * SCALE_FACTOR });
	spike2.setPoint(2, { p3.x * SCALE_FACTOR, -p3.y * SCALE_FACTOR });

	spike2.setPosition({convertToSFML({ 79.0f, 0.0f }, resolution)});

	// Spike3
	p3 = { 1.0f, 6.0f };

	vertices[2] = p3;
	spikeHull = b2ComputeHull(vertices, 3);

	spike = b2MakePolygon(&spikeHull, 0.0f);

	spikeBodyDef.position = { 81.0f, 0.0f };
	b2BodyId spike3BodyId = b2CreateBody(worldId, &spikeBodyDef);

	spikeShapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape(spike3BodyId, &spikeShapeDef, &spike);

	sf::ConvexShape spike3;
	spike3.setPointCount(3);
	spike3.setFillColor(sf::Color::White);

	spike3.setPoint(0, { p1.x * SCALE_FACTOR, -p1.y * SCALE_FACTOR });
	spike3.setPoint(1, { p2.x * SCALE_FACTOR, -p2.y * SCALE_FACTOR });
	spike3.setPoint(2, { p3.x * SCALE_FACTOR, -p3.y * SCALE_FACTOR });

	spike3.setPosition({ convertToSFML({ 81.0f, 0.0f }, resolution) });

	//
	// END OF WORLD SECTION
	
	// View initialization
	applyDefaultView(resolution, window);

	// Start the game loop
	while (window.isOpen())
	{
		// Process events
		while (const std::optional event = window.pollEvent())
		{
			// Disable all the motors at the beginning of the iteration so they don't keep applying forces
			b2RevoluteJoint_EnableMotor(Ids.pivotjointId, false);
			b2RevoluteJoint_EnableMotor(Ids.wheel1jointId, false);
			b2RevoluteJoint_EnableMotor(Ids.wheel2jointId, false);
			// Close window: exit
			if (event->is<sf::Event::Closed>())
				window.close();
		}
		// Pivot Joint Motor
		bool pivotUp = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Up);
		bool pivotDown = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Down);

		if (pivotUp) {
			b2RevoluteJoint_SetMotorSpeed(Ids.pivotjointId, -armwheels.pivotSpeed);
			b2RevoluteJoint_EnableMotor(Ids.pivotjointId, true);
		}
		else if (pivotDown) {
			b2RevoluteJoint_SetMotorSpeed(Ids.pivotjointId, armwheels.pivotSpeed);
			b2RevoluteJoint_EnableMotor(Ids.pivotjointId, true);
		}
		else {
			// If no key is pressed turn off the motor
			b2RevoluteJoint_EnableMotor(Ids.pivotjointId, false);
		}
		// Right Wheel Motor
		bool RwheelRight = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::M);
		bool RwheelLeft = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::N);

		if (RwheelRight) {
			b2RevoluteJoint_SetMotorSpeed(Ids.wheel1jointId, -armwheels.RwheelSpeed);
			b2RevoluteJoint_EnableMotor(Ids.wheel1jointId, true);
		}
		else if (RwheelLeft) {
			b2RevoluteJoint_SetMotorSpeed(Ids.wheel1jointId, armwheels.RwheelSpeed);
			b2RevoluteJoint_EnableMotor(Ids.wheel1jointId, true);
		}
		else {
			// If no key is pressed turn off the motor
			b2RevoluteJoint_EnableMotor(Ids.wheel1jointId, false);
		}

		// Left Wheel Motor
		bool LwheelRight = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::B);
		bool LwheelLeft = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::V);

		if (LwheelRight) {
			b2RevoluteJoint_SetMotorSpeed(Ids.wheel2jointId, -armwheels.LwheelSpeed);
			b2RevoluteJoint_EnableMotor(Ids.wheel2jointId, true);
		}
		else if (LwheelLeft) {
			b2RevoluteJoint_SetMotorSpeed(Ids.wheel2jointId, armwheels.LwheelSpeed);
			b2RevoluteJoint_EnableMotor(Ids.wheel2jointId, true);
		}
		else {
			// If no key is pressed turn off the motor
			b2RevoluteJoint_EnableMotor(Ids.wheel2jointId, false);
		}
		// Losing condition (you lose if fall down two views)
		if (convertToSFML(b2Body_GetPosition(Ids.arm1Id), resolution).y > resolution.y) {

			window.clear();
			applyDefaultView(resolution, window);
			sf::Font font("arial.ttf");
			sf::Text text(font);
			text.setString("You Lose");
			text.setCharacterSize(100);
			text.setFillColor(sf::Color::Red);
			text.setStyle(sf::Text::Bold);
			
			sf::FloatRect bounds = text.getLocalBounds();
			sf::Vector2f origin = bounds.getCenter();
			text.setOrigin(origin);
			text.setPosition({ resolution.x / 2.0f, resolution.y / 2.0f });
			window.draw(text);
			window.display();
		}
		else {
			// Adjust the view (to move the view so that it follows the pivot of the Armwheels)
			adjustView(Ids, resolution, window);
			// printDebug(Ids);
			printArmwheels(Ids, armwheels, worldId, window, resolution);	// Function that renders the Armwheels inside the window in its correct position
			window.draw(ground);
			window.draw(box);
			window.draw(wall);
			window.draw(ground2);
			window.draw(spike1);
			window.draw(spike2);
			window.draw(spike3);
			// Display the drawings (including the world bodies)
			window.display();
			b2World_Step(worldId, dt, substep);
		}
	}
	
}