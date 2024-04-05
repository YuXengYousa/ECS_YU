/*
2024/4/4 NOTE: class Commands ���������� unique_ptr �Ŀ������죬���ù��캯���ѱ�ɾ��
2024/4/4 NOTE: class Commands �� member world(val) �����˿������죬�����շ� unique_ptr �Ŀ������죬����Ϊ��������(ref)
2024/4/4 NOTE: world_.entities_.emplace(entity, World::ComponentContainer()); �˴�������ʽ����� elem ��
               ���� unordered_map �޷����о͵ع��� elem
2024/4/4 TODO��ԭ����ʹ�� world_.entities_.emplace(entity) ����ԭ���д�����
2024/4/5 NOTE: unordered_map::emplace()���ز������cpp reference���ܣ����Խ�����ϵĲ�������
2024/4/5 NOTE: �Ż�ʵ���� Entity,Resource �ӳ����٣����ж� Resource ���ӳ�����ʹ������ԭ���߲�ͬ�����
2024/4/5 TODO: �ƻ�ʵ�� Entity �ӳٴ������漰���� Component ���Ͳ����Ĺ��졢��������ֵ��Ϣ
*/
#pragma once

#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <vector>
#include <optional>
#include <functional>

#include "sparse_sets.hpp"

#define assertm(msg, expr) assert(((void)msg, expr))

namespace ecs {

using ComponentId = uint32_t;
using Entity = uint32_t;

struct Component {};
struct Resource {};

template<typename Category>
class IndexGetter final {
 public:
  template <typename T>
  static uint32_t Get() {
    static uint32_t id_ = cur_idx_++;
    return id_;
  }

 private:
  inline static uint32_t cur_idx_ = 0;
};

template <typename T, typename = std::enable_if<std::is_integral_v<T>>>
class IDGenerator final {
 public:
  static T Gen() { return cur_id_++; }

 private:
  inline static T cur_id_ = {};
};

template<typename T>
class EventStaging final {
 public:
  static void Set(const T& t) { event_ = t; }
  
  static void Set(T&& t) { event_ = std::move(t); }

  static T& Get() {  return *event_; }

  static bool Has() { return event_ != std::nullopt; }

  static void Clear() { event_ = std::nullopt; }

 private:
   inline static std::optional<T> event_ = std::nullopt;
};

template<typename T>
class EventReader final {
 public:
   bool Has() { return EventStaging<T>::Has(); }
    
   const T& Get() { return EventStaging<T>::Get(); }

   void Clear() { EventStaging<T>::Clear(); }

   operator bool() { return EventStaging<T>::Has(); }
};

class World;

class Events final {
 public:
  friend class World;

  template<typename T>
  friend class EventWriter;

  template<typename T>
  auto Reader();

  template<typename T>
  auto Writer();

 private:
  std::vector<std::function<void (void)>> add_event_funcs_;
  std::vector<void (*)(void)> remove_event_funcs_;

  void addAllEvents() {
    for (auto func : add_event_funcs_) {
      func();
    }
    add_event_funcs_.clear();
  }

  void removeEventRead() {
    for (auto func : remove_event_funcs_) {
      func();
    }
    remove_event_funcs_.clear();
  }
};

template<typename T>
class EventWriter final {
 public:
  EventWriter(Events& e) : events_(e) {}
 
  void Write(const T& t);

  // WirteImmediate

 private:
   Events& events_;
};

template <typename T>
auto Events::Reader() {
   remove_event_funcs_.push_back([](){ EventStaging<T>::Clear(); });
   return EventReader<T> {};
}

template<typename T>
auto Events::Writer() {
  return EventWriter<T> {};
}

template<typename T>
void EventWriter<T>::Write(const T& t) {
  events_.add_event_funcs_.push_back([=]() { EventStaging<T>::Set(t); });
}


using EntityGenerator = IDGenerator<Entity>;

class Commands;
class Resources;
class Queryer;

using StartUpSystem = void(*)(Commands&);
using UpdateSystem = void(*)(Commands&, Queryer, Resources, Events&); 

class World final {
 public:
  friend class Commands;
  friend class Resources;
  friend class Queryer;

  using ComponentContainer = std::unordered_map<ComponentId, void*>;

  World() = default;
  World(const World&) = delete;
  World& operator=(const World&) = delete;

  // ����� World ��ʼ��ʱ�����õ� System
  World& AddStartUpSystem(StartUpSystem sys) {
    startup_systems_.push_back(sys);
    return *this;
  }

  // ����� World ����ʱ�����õ� System
  World& AddUpdateSystem(UpdateSystem sys) {
    update_systems_.push_back(sys);
    return *this;
  }

  inline void StartUp();
  inline void Update();
  
  void ShutDown() {
    entities_.clear();
    resources_.clear();
    component_map_.clear();
  }

  template<typename T>
  World& SetResource(T resource);

 private:
  struct Pool final {
    std::vector<void*> instances;
    std::vector<void*> cache;

    using CreateFunc = void* (*)(void);
    using DestroyFunc = void (*)(void*);

    CreateFunc create;
    DestroyFunc destroy;

    Pool(CreateFunc create, DestroyFunc destroy)
        : create(create), destroy(destroy) {}

    void* Create() {
      if (!cache.empty()) {
        instances.push_back(cache.back());
        cache.pop_back();
        return instances.back();
      } else {
        instances.push_back(create());
        return instances.back();
      }
    }

    void Destroy(void* elem) {
      if (auto it = std::find(instances.begin(), instances.end(), elem);
          it != instances.end()) {
        cache.push_back(elem);
        std::swap(*it, instances.back());
        instances.pop_back();
      } else {
        assertm("Your element not in pool", false);
      }
    }
  };

  struct ComponentInfo {
    Pool pool;
    SpareseSets<Entity, 32> sparse_set;

    ComponentInfo(Pool::CreateFunc create, Pool::DestroyFunc destroy)
        : pool(create, destroy) {}

    ComponentInfo() : pool(nullptr, nullptr) {}
  };

  using ComponentMap = std::unordered_map<ComponentId, ComponentInfo>;
  ComponentMap component_map_;
  std::unordered_map<Entity, ComponentContainer> entities_;

  struct ResourceInfo {
    void* resource = nullptr;
    using DestroyFunc = void(*)(void*);
    DestroyFunc destroy { nullptr };

    ResourceInfo() = default;

    ResourceInfo(DestroyFunc destroy)
        : destroy(destroy) {
      assertm("You must give a not-null destory function!", destroy);

    }

    ~ResourceInfo() {
      destroy(resource);
    }
  };

  std::unordered_map<ComponentId, ResourceInfo> resources_;
  std::vector<StartUpSystem> startup_systems_;
  std::vector<UpdateSystem> update_systems_;
  Events events_;
};


class Commands final {
 public:
  Commands(World& world) : world_(world) {}

  template<typename... ComponentTypes>
  Commands& Spawn(ComponentTypes&&... components) {
    SpwanAndReturn<ComponentTypes...>(std::forward<ComponentTypes>(components)...);
    return *this;
  }

  template<typename... ComponentTypes>
  Entity SpwanAndReturn(ComponentTypes&&... components) {
    Entity entity = EntityGenerator::Gen();
    doSpawn(entity, std::forward<ComponentTypes>(components)...);  // ����ת��
    return entity;
  }

  Commands& Destroy(Entity entity) {
    destory_entitise_.push_back(entity);

    return *this;
  }

  template<typename T>
  Commands& SetResource(T&& resource) {
    auto index = IndexGetter<Resource>::Get<T>();
    if (auto it = world_.resources_.find(index);
        it != world_.resources_.end()) {
      // ������������
      assertm("Resource already exists.", it->second.resource);
      it->second.resource = new T(std::forward<T>(resource));
    } else {
      auto other_it = world_.resources_.emplace(
        index, World::ResourceInfo([](void* elem){ delete (T*)elem; }));
      other_it.first->second.resource = new T(std::forward<T>(resource));
      *((T*)other_it.first->second.resource) = std::forward<T>(resource);
      // std::pair<std::_List_iterator<std::_List_val<std::_List_simple_types<std::pair<unsigned int const ,ecs::World::ResourceInfo>>>>,bool>

    }

    return *this;
  }

  template<typename T>
  Commands& RemoveResource() {
    auto index = IndexGetter<Resource>::Get<T>();
    destroy_resources_.push_back(index);
    //if (auto it = world_.resources_.find(index);
    //    it != world_.resources_.end()) {
    //  delete (T*)it->second.resource; // ע��ǿ������ת��
    //  it->second.resource = nullptr;
    //}
    return *this;
  }
  
  void Execute() {
    for (auto res : destroy_resources_) {
      destroyResource(res);
    }
    for (auto e : destory_entitise_) {
      destroyEntity(e);
    }
    destroy_resources_.clear();
    destory_entitise_.clear();
  }

 private:
  World& world_;
  
  // ������ԭ���ߵ������������
  std::vector<ComponentId> destroy_resources_;
  // ʵ������ٻ���
  std::vector<Entity> destory_entitise_;


  void destroyResource(ComponentId id) {
    if (auto it = world_.resources_.find(id);
        it != world_.resources_.end()) {
      it->second.destroy(it->second.resource); 
      it->second.resource = nullptr;
    }
  }

  void destroyEntity(Entity entity) {
    if (auto it = world_.entities_.find(entity); it != world_.entities_.end()) {
      for (auto [id, component] : it->second) {
        auto& component_info = world_.component_map_[id];
        component_info.pool.Destroy(component);
        component_info.sparse_set.Remove(entity);
      }
      world_.entities_.erase(it);
    }
  }

  template <typename T, typename... Remains>  // �ɱ��������
  void doSpawn(Entity entity, T&& component, Remains&&... remains) {
    auto index = IndexGetter<Component>::Get<T>();
    if (auto it = world_.component_map_.find(index);
        it == world_.component_map_.end()) {
      world_.component_map_.emplace(
          index, World::ComponentInfo([]() -> void* { return new T; },
                                      [](void* elem) { delete (T*)(elem); }));
    }
    World::ComponentInfo& component_info = world_.component_map_[index];
    void* elem = component_info.pool.Create();
    (*(T*)elem) = std::forward<T>(component);  // ����ת��
    component_info.sparse_set.Add(entity);

    auto it =
        world_.entities_.emplace(entity, World::ComponentContainer());
    it.first->second[index] = elem;
    // ����ʱ�����ж�
    if constexpr (sizeof...(remains) != 0) {
      doSpawn<Remains...>(entity, std::forward<Remains>(remains)...);
    }
  }
};

class Resources final {
 public:
  Resources(World& world) : world_(world) {}

  template<typename T>
  bool Has() const {
    auto index = IndexGetter<Resources>::Get<T>();
    auto it = world_.resources_.find(index);
    return it != world_.resources_.end() && it->second.resource;
  }

  template<typename T>
  T& Get() {
    auto index = IndexGetter<Resources>::Get<T>();
    return *((T*)world_.resources_[index].resource);
  }
 private:
  World& world_;
};

class Queryer final {
 public:
  Queryer(World& world) : world_(world) {}

  template<typename...Components>
  std::vector<Entity> Query() {
    std::vector<Entity> entities;
    doQuery<Components...>(entities);
    return entities;
  }

  // ��� entty �Ƿ�ӵ��ָ�����͵� Component
  template<typename T>
  bool Has(Entity entity) {
    auto it = world_.entities_.find(entity);
    auto index = IndexGetter<Component>::Get<T>();
    return it != world_.entities_.end() &&
           it->second.find(index) != it->second.end();
  }

  // ��ȡ entity ��ָ�����͵� Component��Ҫ��ʹ�� Get ��� entity �Ƿ����
  template<typename T>
  T& Get(Entity enity) {
    auto index = IndexGetter<Component>::Get<T>();
    return *((T*)world_.entities_[enity][index]);
  }

 private:
   World& world_;

  template<typename T, typename...Remains>
  bool doQuery(std::vector<Entity>& outEntities) {
    auto index = IndexGetter<Component>::Get<T>();
    World::ComponentInfo& component_info = world_.component_map_[index];
    for (auto e : component_info.sparse_set) {
      if constexpr (sizeof...(Remains) > 0) {
        doQueryRemains<Remains...>(e, outEntities);
      } else {
        // ����������в�������Ϊ 0 �����
        outEntities.push_back(e);
      }
    }
    return !outEntities.empty();
  }

  template<typename T, typename...Remains>
  bool doQueryRemains(Entity entity, std::vector<Entity>& outEntities) {
    auto index = IndexGetter<Component>::Get<T>();
    World::ComponentContainer& container = world_.entities_[entity];
    // ��� entity ��ӵ��Ŀ�� component
    if (container.find(index) == container.end()) {
      return false;
    }
    // ������չ����ɣ�entity ӵ�в������ڵ��������͵� component
    if constexpr (sizeof...(Remains) == 0) {
      outEntities.push_back(entity);
      return true;
    } else {
      return doQueryRemains<Remains...>(entity, outEntities);
    }
  }
};

inline void World::StartUp() {
  Commands command(*this);
  for (auto sys : startup_systems_) {
    sys(command);
  }
  command.Execute();
}

inline void World::Update() {
  Commands command(*this);
  for (auto sys : update_systems_) {
    sys(command, Queryer(*this), Resources(*this), events_);
  }
  command.Execute();

  events_.removeEventRead();
  events_.addAllEvents();
}

template <typename T>
World& World::SetResource(T resource) {
  Commands commands(*this);
  commands.SetResource<T>(std::forward<T>(resource));

  return *this;
}

}  // namespace ecs
