
/* Singleton 是单例类的模板.
 * https://www.cnblogs.com/sunchaothu/p/10389842.html
 * 单例 Singleton 是设计模式的一种, 其特点是只提供唯一一个类的实例,具有全局变量的特点, 在任何位置都可以通过接口获取到那个唯一实例;
 * 全局只有一个实例: static 特性, 同时禁止用户自己声明并定义实例（把构造函数设为 private 或者 protected）
 * Singleton 模板类对这种方法进行了一层封装.
 * 单例类需要从Singleton继承.
 * 子类需要将自己作为模板参数T 传递给 Singleton<T> 模板;
 * 同时需要将基类声明为友元, 这样Singleton才能调用子类的私有构造函数.
// 子类必须把父类设定为友元函数, 这样父类才能使用子类的私有构造函数.
// 父类的构造函数必须保护, 子类的构造函数必须私有.
// 必须关闭拷贝构造和赋值构造, 只能通过 get_instance 函数来操作 唯一的实例.
 * */

#ifndef __SINGLIETON_H
#define __SINGLIETON_H

#include <utility>

namespace Interface {
namespace common {
template<typename T, typename ...BUILD_ARGS>
class Singleton {
public:
    /**
     * @brief 获取单例的引用, 提供构造函数需要参数, 在第一次调用时传入参数
     * @param args 构造函数参数
     */
    template<typename... Args>
    static T &get_instance_references(Args &&... args) {
        return *iter(std::forward<Args>(args)...);
    }

    /**
     * @brief 获取单例的指针, 提供构造函数需要参数, 在第一次调用时传入参数
     * @param args 构造函数参数
     */
    template<typename... Args>
    static T *get_instance_pointer(Args &&... args) {
        return iter(std::forward<Args>(args)...);
    }

    /**
     * @brief 获取单例的指针, 提供构造函数需要参数, 在第一次调用时传入参数
     * @param args 构造函数参数
     */
    template<typename... Args>
    static T *iter(Args &&... args) {
        if (static_instance() == nullptr) {
            static_instance(new T(std::forward<Args>(args)...));  //如果实例不存在, 则创建一个新的实例
        }
        return static_instance();
    }

    virtual ~Singleton() {}

    Singleton(const Singleton &) = delete;                    //关闭拷贝构造函数
    Singleton &operator=(const Singleton &) = delete;        //关闭赋值函数

protected:
    //构造函数需要是 protected, 这样子类才能继承；
    Singleton() {}

    /**
     * @brief 静态实例函数, 必须私有, 存放唯一的实例指针
     * @param instance 如果传入实例, 则返回传入的实例, 否则返回静态实例指针
     */
    static T* static_instance(T *instance = nullptr) {
        static T *instance_ = nullptr;  //单例的实例指针
        if (instance_ == nullptr) {
            instance_ = instance;
        }
        return instance_;
    }

};

/*
// 如下是 使用样例: 
// 子类必须把父类设定为友元函数, 这样父类才能使用子类的私有构造函数.
// 父类的构造函数必须保护, 子类的构造函数必须私有.
// 必须关闭拷贝构造和赋值构造, 只能通过 get_instance 函数来进行操作唯一的实例.

class DerivedSingle:public Singleton<DerivedSingle>
{
 // 子类必须把父类设定为友元函数, 这样父类才能使用子类的私有构造函数.
	friend class Singleton<DerivedSingle>;
public:
 // 必须关闭拷贝构造和赋值构造, 只能通过 get_instance 函数来进行操作唯一的实例.
	DerivedSingle(const DerivedSingle&)=delete;
	DerivedSingle& operator =(const DerivedSingle&)= delete;
 	~DerivedSingle()=default;
private:
 // 父类的构造函数必须保护, 子类的构造函数必须私有.
	DerivedSingle()=default;
};

int main(int argc, char* argv[]){
	DerivedSingle& instance1 = DerivedSingle::get_instance_references();
	DerivedSingle* p_instance2 = DerivedSingle::get_instance_pointer();
	return 0;
}

 */

} // namespace common
} // namespace CombinedInterface
#endif