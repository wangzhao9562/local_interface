#pragma once

/* TODO: Singleton realization for single thread application
 */
template<class T>
class CSingletonST{
private:
  static T* instance_;
  
  CSingletonST(){};

public:
  /* Get single instance 
   * @return ptr of single instance
   */
  static T* GetInstance(){
    if(instance_ == nullptr){
      instance_ = new T();
    }
    return instance_;
  }
};

/* TODO: Singleton realization for multi thread application
 */
template<class T>
class CSingletonMT{
private:
  static T* instance_;
  CSingletonMT(){};

public:
  /* Get single instance
   * @return ptr of single instance
   */
  static T* GetInstance(){
    return instance_;
  }
};

template<class T>
T* CSingletonMT<T>::instance_ = new T();
