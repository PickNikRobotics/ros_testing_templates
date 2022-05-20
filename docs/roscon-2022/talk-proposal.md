ROSCon 2022 Talk Proposal
Talk proposals must be submitted by 2022-06-09

<!-- Title (maximum 70 characters) -->
# Getting ROS out of your robot, or how I learned to love dependency injection

## Presenter
Griswald Brooks - PickNik Robotics

## Desired talk length
30 minutes

## Summary - for public consumption, used in the program schedule (maximum 100 words)
ROS solves **many** problems for robot software developers, but naive use of it
can make true (or some other term to rebutts the idea that all tests in ros are "unit test")
unit testing challenging. A layered testing approach, with unit
tests that cannot be flaky (need a less awkward way to phrase this) as the base,
are essential to writing maintainable software. This talk will present the
dependency injection software design pattern, as it pertains to ROS, to convert
naive ROS tests into mocked unit tests. We have also released `ros_testing_templates`,
a repository of testing examples which demonstrates this pattern.

## Description, for review by the program committee
Please be sure to include enough information in your proposal for the program committee to evaluate the above review criteria.

- goals
  - describe the benefits of unit testing over integration testing
  - mention that composition is good, `friend` is bad
  - show dependency injection pattern and it's use with mocking
    - SOLID (separation of concerns, portability?), reduce testing scope
  - Something about BDD? Short tests? Parameterized tests?
  - present `ros_testing_templates`
- intended audience and what they can expect to learn
  - developers in a production environment, learning a pattern to improve software quality

### brief summary
- Relevance to the ROS Community
  - Talk is focused on how to use ROS... responsibly.
  - Is applicable to all scales of project
  - Discusses a pattern to improve maintainability, by lowering unit test friction
- Quality of Content/Impact
  - di'ing your code -> increasing testing -> 10x your quality
- Quality of Presentation
  - I am best at present
- Originality/Novelty
  - ROS1 promoted gtest/gmock, and everyone says testing is good, but i haven't
    heard about a talk specifically on this and I've yet to meet a ROS developer
    (outside of bossanova) who knew how to do this in a matter of fact way
- Open Source Availability
  - releasing (is that the right work here since it's been public?) a testing
    examples repo for ros1/ros2 that will have ci and development containers so
    it can be easily cloned and tried

### outline
- Describe the problem with testing, mocking, dependency injection, results?
- Define unit, integration, simulation, onrobot testing
- what is mocking?
- dependency injection is not a new technique, but it's a design pattern that
  requires forethought, best done in the beginning of a project
  - reference C++ talks/links on DI, gmock, anti-singleton talk, `boost::di`
- (ros has many subtle ways to weave itself into your code, making unit testing hard)
- (pub/sub, actions, service, parameters, time, rate, ros::ok)
- Pitfalls
  - limit di to one level (ie, as shallow as possible), deep di (service locator?)
    can make chasing down regressions confusing
  - treat di as capturing the other public interface of your code (ie return statements), capturing side effects (but the actual ones). Don't use it like
  member variables that hold state
  - Composed/custom di vs reusable di
  - `auto` makes this nice...
  - Can we get our value semantics back? Type Erasure
- `ros_testing_templates` should show not only di, but integration tests, sim test? composition pattern, functional vs oo, functions vs methods? feels like a best practices repo. probably should show what it takes to test each style
  - README should link to talks and have lots of description of contained patterns
  - ros1 and ros2
- `development-container` shoutout

## Audio abstract - All talk proposals must include a (maximum) one minute recording of the presenter describing the content of the talk. It should be a single-take, responding as if a colleague asked what the talk was about. Please use either the mp3 or ogg file format. Used only for review by the Program Committee (not made public).
I am best speak well

## Key URL/Twitter handles - Optionally include a single link/Twitter handles to be associated with talk in publicity materials.
https://twitter.com/picknikrobotics
https://github.com/PickNikRobotics/ros_testing_templates

## (NEW) In-person vs. remote: We prefer in-person presentations, but we will consider allowing remote presentations as needed.
In-person
