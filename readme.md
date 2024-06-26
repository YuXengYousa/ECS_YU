# ECS_YU
## 什么是 ECS?
允许我引用著名 ECS 框架[Entt](https://github.com/skypjack/entt)对 ECS 的介绍...
> The entity-component-system (also known as ECS) is an architectural pattern used mostly in game development. For further details:

实体-组件-系统（也称为 ECS ）是一种主要用于游戏开发的架构模式。详情如下：
* [Entity Systems Wiki](http://entity-systems.wikidot.com/)
* [Evolve Your Hierarchy](http://cowboyprogramming.com/2007/01/05/evolve-your-heirachy/)
* [ECS on Wikipedia](https://en.wikipedia.org/wiki/Entity%E2%80%93component%E2%80%93system)

ECS 是一种主要用于游戏开发的架构模式。ECS 遵循 Composition over Inheritance （即**组合优于继承**） 原则，通过构建可以混合搭配的单个部分，在定义实体（游戏场景中的任何内容：敌人、门、子弹）时具有更大的灵活性。这消除了长继承链的模糊性问题，并促进了简洁的设计。无论如何，ECS 系统的性能成本确实很小。
>引自 Entity Systems Wiki
## 关于本项目
是对 Entt 等 ECS 框架的简单模仿，使用 C++ 并结合其特性，意图实现 ECS 框架的一些 API 。目前仍在 Coding 阶段，参考学习的资料链接之后会整理在这里，欢迎共同学习。