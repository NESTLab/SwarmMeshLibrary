#ifndef FACTORY_H
#define FACTORY_H

#include <map>
#include <iostream>
#include <string>
#include <cstdlib>


/**
* Basic factory template
*/
template<typename TYPE>
class CFactory {

public:
    /**
     * Pointer to the function that creates objects of type <tt>TYPE</tt>
     */
    typedef TYPE* TCreator();

    /**
     * A struct containing the information about the registered types
     */
    struct STypeInfo {
        TCreator* Creator;
        std::string Author;

    };

    /**
     * The map of registered <tt>TYPE</tt>s
     */
    typedef std::map<std::string, STypeInfo*> TTypeMap;

public:
    /**
     * Creates and returns the <tt>TYPE</tt> map
     * @return The <tt>TYPE</tt> map
     */
    static TTypeMap& GetTypeMap();

    /**
     * Registers a new <tt>TYPE</tt> creator in the factory
     * @param str_label The label associated to the <tt>TYPE</tt> creator
     * @param str_author The author of the plugin
     * @param pc_creator The <tt>TYPE</tt> creator of the factory
     */
    static void Register(const std::string& str_label,
                        const std::string& str_author,
                        TCreator* pc_creator);

    /**
     * Creates a new object of type <tt>TYPE</tt>
     * @param str_label The label of the <tt>TYPE</tt> to create
     * @return A new object of type <tt>TYPE</tt>
     */
    static TYPE* New(const std::string& str_label);

    /**
     * Returns <tt>true</tt> if the given label exists in the <tt>TYPE</tt> map
     * @return <tt>true</tt> if the given label exists in the <tt>TYPE</tt> map
     */
    static bool Exists(const std::string& str_label);

    /**
     * Prints the list of registered labels
     * @param c_os The <tt>std::ostream</tt> to write into
     */
    static void Print(std::ostream& c_os);

    /**
     * Frees up all used memory.
     */
    static void Destroy();
};

#endif