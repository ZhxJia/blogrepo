---
title: Component 组件注册过程
categories:
- apollo
tags:
- apollo
---

<!-- more -->

```ｃ++
CYBER_REGISTER_COMPONENT(FusionCameraDetectionComponent);
```

```c++
#define CYBER_REGISTER_COMPONENT(name) \
  CLASS_LOADER_REGISTER_CLASS(name, apollo::cyber::ComponentBase)
```

```c++
// register class macro
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, __COUNTER__)

```

```c++
#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, UniqueID) \
  CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)

```

最终实际执行：
```c++
#define CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)     \
  namespace {                                                             \
  struct ProxyType##UniqueID {                                            \
    ProxyType##UniqueID() {                                               \
      apollo::cyber::class_loader::utility::RegisterClass<Derived, Base>( \
          #Derived, #Base);                                               \
    }                                                                     \
  };                                                                      \
  static ProxyType##UniqueID g_register_class_##UniqueID;                 \
  }

```



```c++

template <typename Derived, typename Base>
void RegisterClass(const std::string& class_name,
                   const std::string& base_class_name) {
  AINFO << "registerclass:" << class_name << "," << base_class_name << ","
        << GetCurLoadingLibraryName();

  utility::AbstractClassFactory<Base>* new_class_factrory_obj =
      new utility::ClassFactory<Derived, Base>(class_name, base_class_name);
  new_class_factrory_obj->AddOwnedClassLoader(GetCurActiveClassLoader());
  new_class_factrory_obj->SetRelativeLibraryPath(GetCurLoadingLibraryName());

  GetClassFactoryMapMapMutex().lock();
  ClassClassFactoryMap& factory_map =
      GetClassFactoryMapByBaseClass(typeid(Base).name());
  factory_map[class_name] = new_class_factrory_obj;
  GetClassFactoryMapMapMutex().unlock();
}
```

