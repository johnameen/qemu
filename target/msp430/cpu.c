/**
 * TODO: Insert Copyright and Attribution Here
 */
#include "cpu.h"
#include "exec/exec-all.h"
// #include "qemu-common.h"
#include "exec/cpu_ldst.h"
#include "exec/log.h"
#include "qapi/error.h"
#include "hw/loader.h"

// TODO: Is this okay?
#include "translate.h"

#define MSP430_NUM_CPUS              (sizeof(MspCpus)/sizeof(MSP430CpuInfo))

typedef struct {
    const char *name;
    void (*initfn)(Object *obj);
    void (*class_init)(ObjectClass *klass, void *data);
} MSP430CpuInfo;

/********************************************************************************
 * Prototype Functions (Private)
 *******************************************************************************/
static void msp430_init(Object *obj);


/********************************************************************************
 * Global Functions
 *******************************************************************************/
/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param s [description]
 */
static void msp430_cpu_reset(DeviceState *dev)
{
    CPUState *s = CPU(dev);
    MSP430Cpu *cpu = MSP430_CPU(s);
    MSP430Class *msp430_class = MSP430_CPU_GET_CLASS(cpu);
    MSP430CpuState *state = &cpu->env;
    
    msp430_class->parent_reset(dev);
    memset(state, 0, sizeof(MSP430CpuState));
    
    uint8_t *rom = rom_ptr(0xFFFE, 2);
    uint32_t resetAddr = (0xFFFE & ldl_p(rom));
    state->regs[MSP430_PC_REGISTER] = resetAddr;
    tlb_flush(s);
}

static void msp430_cpu_debug_excp_handler(CPUState *s) { log_cpu_state(s, 0); }


/**
 * @brief Callback function for CpuClass to set the PC value
 * @details Callback function for CpuClass to set the PC value. This function
 * is used in order for CpuObjects to implement how to set the PC value
 * 
 * @param cs CPUState object to set the value to.
 * @param value The value to set the PC.
 */
static void msp430_set_pc(CPUState *cs, vaddr value)
{
    MSP430Cpu *cpu = MSP430_CPU(cs);
    cpu->env.PC_REG = (0xFFFF & value);
}

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param cpu_model [description]
 * @param resetAddr [description]
 * 
 * @return [description]
 */
MSP430Cpu *cpu_msp430_init(const char *cpu_model)
{
    ObjectClass *oc = cpu_class_by_name(TYPE_MSP430_CPU, cpu_model);
    if (!oc){
        return NULL;
    }
    //    MSP430Cpu *cpu = MSP430_CPU(msp430_class_by_name(cpu_model));
    MSP430Cpu *cpu = MSP430_CPU(oc);
    cpu_reset(CPU(cpu));
    return cpu;
}

/********************************************************************************
 * Global Data Structures
 *******************************************************************************/
static const MSP430CpuInfo MspCpus[] = {
    { .name = "msp430", .initfn = msp430_init}, 
};

static ObjectClass *msp430_class_by_name(const char *cpu_model)
{
    ObjectClass *oc;

    if (cpu_model == NULL) {
        return NULL;
    }

    oc = object_class_by_name(cpu_model);
    if (oc != NULL && (!object_class_dynamic_cast(oc, TYPE_MSP430_CPU) ||
                       object_class_is_abstract(oc))) {
        return NULL;
    }
    return oc;
}

/**
 * @brief MSP430 Object Initialization
 * @details Initialization function for a new MSP430 CPU object. In this function
 * we initialize the CPU to an initial state, as well as other things such as 
 * the translator engine.
 * 
 * @param obj MSP430Cpu object that is being initialized
 */
static void msp430_init(Object *obj)
{
    MSP430Cpu *cpu = MSP430_CPU(obj);
    // CPUState *cpu_state = CPU(obj);
    
    /* Fetch the proper states and initialize the CPU and the parent
     * CPU object to be initialized. */
    cpu_set_cpustate_pointers(cpu);

    return;
}

static void msp430_cpu_realize(DeviceState *dev, Error **errp){
    CPUState *cs = CPU(dev);
    MSP430Class *msp430_class = MSP430_CPU_GET_CLASS(dev);
    qemu_init_vcpu(cs);
    msp430_class->parent_realize(dev, errp);
}

static void cpu_register(const MSP430CpuInfo *info)
{
    TypeInfo type_info = {
        .parent = TYPE_MSP430_CPU,
        .instance_size = sizeof(MSP430Cpu),
        .instance_init = info->initfn,
        .class_size = sizeof(MSP430Class),
        .class_init = info->class_init,
    };

    type_info.name = g_strdup_printf("%s-" TYPE_MSP430_CPU, info->name);
    type_register(&type_info);
    g_free((void *)type_info.name);
}


// static void msp430_cpu_synchronize_from_tb(CPUState *cs,
//                                            const TranslationBlock *tb)
// {
//     MSP430Cpu *cpu = MSP430_CPU(cs);

//     cpu->env.PC_REG = tb->pc;
// }

#ifdef CONFIG_TCG
#include "hw/core/tcg-cpu-ops.h"

static const struct TCGCPUOps msp430_tcg_ops = {
    .initialize = msp430_tcg_init,
    // .synchronize_from_tb = msp430_cpu_synchronize_from_tb,
    .debug_excp_handler = msp430_cpu_debug_excp_handler,

#ifndef CONFIG_USER_ONLY
    .tlb_fill = msp430_tlb_fill,
    // .cpu_exec_interrupt = msp430_cpu_exec_interrupt,
    // .do_interrupt = msp430_cpu_do_interrupt,
    // .do_transaction_failed = msp430_cpu_do_transaction_failed,
    // .do_unaligned_access = msp430_cpu_do_unaligned_access,
#endif /* !CONFIG_USER_ONLY */
};
#endif /* CONFIG_TCG */

static void msp430_cpu_class_init(ObjectClass *oc, void *data)
{
    MSP430Class *mc = MSP430_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    DeviceClass *dc =  DEVICE_CLASS(oc);

    device_class_set_parent_realize(dc, msp430_cpu_realize, &mc->parent_realize);
    // device_class_set_props(dc, msp430_cpu_properties);
    device_class_set_parent_reset(dc, msp430_cpu_reset, &mc->parent_reset);

    cc->class_by_name = msp430_class_by_name;
    cc->set_pc = msp430_set_pc;
    cc->dump_state = msp430_cpu_dump_state;

    cc->tcg_ops = &msp430_tcg_ops;

    return;
}

static const TypeInfo msp430_cpu_type_info = {
    .name = TYPE_MSP430_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(MSP430Cpu),
    .instance_init = msp430_init,
    .class_size = sizeof(MSP430Class),
    .class_init = msp430_cpu_class_init,
};

/**
 * @brief Registers all the CPUs that are available as an MSP430
 * @details Registers the static version of the CPU and performs CPU registrations 
 * on all the different CPU flavors of MSP430
 */
static void msp430_register_cpus(void)
{
    int i = 0;
    type_register_static(&msp430_cpu_type_info);
    for (i=0; i<MSP430_NUM_CPUS; i++)
        cpu_register(&MspCpus[i]);
    return;
}



type_init(msp430_register_cpus)