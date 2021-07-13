/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>

#include <px4_platform_common/px4_work_queue/WorkQueue.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/atomic.h>
#include <containers/BlockingList.hpp>
#include <containers/BlockingQueue.hpp>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>

#include <limits.h>
#include <string.h>

using namespace time_literals;

namespace px4
{

// list of current work queues
static BlockingList<WorkQueue *> *_wq_manager_wqs_list{nullptr};

// queue of WorkQueues to be created (as threads in the wq manager task) 要创建的工作队列队列(作为wq manager任务中的线程)
static BlockingQueue<const wq_config_t *, 1> *_wq_manager_create_queue{nullptr};

static px4::atomic_bool _wq_manager_should_exit{true};

static uint32_t  test_flag_wq_manager_create_queue = 0;
static uint32_t  test_count__ = 0 ;//看看 pop 调用了多少次
static uint32_t  test_WorkQueueRunner__a  = 0;//看看 WorkQueueRunner 调用了多少次
static uint32_t  test_WorkQueueRunner__b  = 0 ;//看看 WorkQueueRunner 调用了多少次


static WorkQueue *
FindWorkQueueByName(const char *name)
{
	if (_wq_manager_wqs_list == nullptr) {
		PX4_ERR("not running");
		return nullptr;
	}

	LockGuard lg{_wq_manager_wqs_list->mutex()};

	// search list
	for (WorkQueue *wq : *_wq_manager_wqs_list) {
		if (strcmp(wq->get_name(), name) == 0) {
			return wq;
		}
	}

	return nullptr;
}

WorkQueue *
WorkQueueFindOrCreate(const wq_config_t &new_wq)
{
	if (_wq_manager_create_queue == nullptr) {
		PX4_ERR("not running");
		return nullptr;
	}

	// search list for existing work queue  在工作队列里查找 是否已注册
	//名字是通过查找出来的 不是外设自定义名字，这个静态名字定义在 platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp
	//名字类似于  wq:SPI0  wq:SPI1 wq:SPI2  wq:I2C0 wq:I2C1
	//查找名字 用函数  device_bus_to_wq 或  serial_port_to_wq
	//new_wq 这个变量 是已经静态生成的！！！
	WorkQueue *wq = FindWorkQueueByName(new_wq.name);

	// 如果之前没有创建 则返回 null  然后通过下面的程序去创建  如果以创建 则返回 WorkQueue *
	// create work queue if it doesn't already exist
	if (wq == nullptr) {
		// add WQ config to list
		//  main thread wakes up, creates the thread
		//_wq_manager_create_queue 队列只有一个元素
		//这个元素在 主线程中 一直被判断是否为空 如果不为空 则创建一个线程
		_wq_manager_create_queue->push(&new_wq);
		//PX4_INFO("%s need create",new_wq.name);
		//等待线程被创建 然后继续运行
		// we wait until new wq is created, then return
		uint64_t t = 0;

		while (wq == nullptr && t < 10_s) {
			// Wait up to 10 seconds, checking every 1 ms
			t += 1_ms;
			px4_usleep(1_ms);
			//如果查找到了 表示创建成功
			wq = FindWorkQueueByName(new_wq.name);
		}

		if (wq == nullptr) {
			PX4_ERR("failed to create %s", new_wq.name);
		}
	}

	return wq;
}

const wq_config_t &
device_bus_to_wq(uint32_t device_id_int)
{
	union device::Device::DeviceId device_id;
	device_id.devid = device_id_int;

	const device::Device::DeviceBusType bus_type = device_id.devid_s.bus_type;
	const uint8_t bus = device_id.devid_s.bus;

	if (bus_type == device::Device::DeviceBusType_I2C) {
		switch (bus) {
		case 0: return wq_configurations::I2C0;

		case 1: return wq_configurations::I2C1;

		case 2: return wq_configurations::I2C2;

		case 3: return wq_configurations::I2C3;

		case 4: return wq_configurations::I2C4;
		}

	} else if (bus_type == device::Device::DeviceBusType_SPI) {
		switch (bus) {
		case 0: return wq_configurations::SPI0;

		case 1: return wq_configurations::SPI1;

		case 2: return wq_configurations::SPI2;

		case 3: return wq_configurations::SPI3;

		case 4: return wq_configurations::SPI4;

		case 5: return wq_configurations::SPI5;

		case 6: return wq_configurations::SPI6;
		}
	}

	// otherwise use high priority
	return wq_configurations::hp_default;
};

const wq_config_t &
serial_port_to_wq(const char *serial)
{
	if (serial == nullptr) {
		return wq_configurations::hp_default;

	} else if (strstr(serial, "ttyS0")) {
		return wq_configurations::UART0;

	} else if (strstr(serial, "ttyS1")) {
		return wq_configurations::UART1;

	} else if (strstr(serial, "ttyS2")) {
		return wq_configurations::UART2;

	} else if (strstr(serial, "ttyS3")) {
		return wq_configurations::UART3;

	} else if (strstr(serial, "ttyS4")) {
		return wq_configurations::UART4;

	} else if (strstr(serial, "ttyS5")) {
		return wq_configurations::UART5;

	} else if (strstr(serial, "ttyS6")) {
		return wq_configurations::UART6;

	} else if (strstr(serial, "ttyS7")) {
		return wq_configurations::UART7;

	} else if (strstr(serial, "ttyS8")) {
		return wq_configurations::UART8;
	}

	PX4_DEBUG("unknown serial port: %s", serial);

	return wq_configurations::UART_UNKNOWN;
}

static void *
WorkQueueRunner(void *context)
{
	wq_config_t *config = static_cast<wq_config_t *>(context);
	WorkQueue wq(*config);

	test_WorkQueueRunner__a++;

	// add to work queue list
	_wq_manager_wqs_list->add(&wq);
	//PX4_INFO("%s run",config->name);
	wq.Run();
	//PX4_INFO("%s run  over",config->name);
	// remove from work queue list
	_wq_manager_wqs_list->remove(&wq);
	test_WorkQueueRunner__b++;
	return nullptr;
}

static int
WorkQueueManagerRun(int, char **)
{

	_wq_manager_wqs_list = new BlockingList<WorkQueue *>();
	_wq_manager_create_queue = new BlockingQueue<const wq_config_t *, 1>();


	while (!_wq_manager_should_exit.load()) {
		// create new work queues as needed  根据需要创建新的工作队列
		const wq_config_t *wq = _wq_manager_create_queue->pop();

		test_count__ = test_count__ +1 ;
		if (wq != nullptr) {
			// create new work queue

			pthread_attr_t attr;
			int ret_attr_init = pthread_attr_init(&attr);
			test_flag_wq_manager_create_queue++;
			if (ret_attr_init != 0) {
				PX4_ERR("attr init for %s failed (%i)", wq->name, ret_attr_init);
			}

			sched_param param;
			int ret_getschedparam = pthread_attr_getschedparam(&attr, &param);

			if (ret_getschedparam != 0) {
				PX4_ERR("getting sched param for %s failed (%i)", wq->name, ret_getschedparam);
			}

			// stack size
#if defined(__PX4_QURT)
			const size_t stacksize = math::max(8 * 1024, PX4_STACK_ADJUSTED(wq->stacksize));
#elif defined(__PX4_NUTTX)
			const size_t stacksize = math::max((uint16_t)PTHREAD_STACK_MIN, wq->stacksize);
#elif defined(__PX4_POSIX)
			// On posix system , the desired stacksize round to the nearest multiplier of the system pagesize
			// It is a requirement of the  pthread_attr_setstacksize* function
			const unsigned int page_size = sysconf(_SC_PAGESIZE);
			const size_t stacksize_adj = math::max(PTHREAD_STACK_MIN, PX4_STACK_ADJUSTED(wq->stacksize));
			const size_t stacksize = (stacksize_adj + page_size - (stacksize_adj % page_size));
#endif
			int ret_setstacksize = pthread_attr_setstacksize(&attr, stacksize);

			if (ret_setstacksize != 0) {
				PX4_ERR("setting stack size for %s failed (%i)", wq->name, ret_setstacksize);
			}

#ifndef __PX4_QURT

			// schedule policy FIFO
			int ret_setschedpolicy = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

			if (ret_setschedpolicy != 0) {
				PX4_ERR("failed to set sched policy SCHED_FIFO (%i)", ret_setschedpolicy);
			}

#endif // ! QuRT

			// priority
			param.sched_priority = sched_get_priority_max(SCHED_FIFO) + wq->relative_priority;
			int ret_setschedparam = pthread_attr_setschedparam(&attr, &param);

			if (ret_setschedparam != 0) {
				PX4_ERR("setting sched params for %s failed (%i)", wq->name, ret_setschedparam);
			}

			// create thread  创建线程
			pthread_t thread;
			int ret_create = pthread_create(&thread, &attr, WorkQueueRunner, (void *)wq);

			if (ret_create == 0) {
				PX4_DEBUG("starting: %s, priority: %d, stack: %zu bytes", wq->name, param.sched_priority, stacksize);

			} else {
				PX4_ERR("failed to create thread for %s (%i): %s", wq->name, ret_create, strerror(ret_create));
			}

			// destroy thread attributes
			int ret_destroy = pthread_attr_destroy(&attr);

			if (ret_destroy != 0) {
				PX4_ERR("failed to destroy thread attributes for %s (%i)", wq->name, ret_create);
			}
		}
	}

	return 0;
}

int
WorkQueueManagerStart()
{
	if (_wq_manager_should_exit.load() && (_wq_manager_create_queue == nullptr)) {

		_wq_manager_should_exit.store(false);

		int task_id = px4_task_spawn_cmd("wq:manager",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX,
						 1280,
						 (px4_main_t)&WorkQueueManagerRun,
						 nullptr);

		if (task_id < 0) {
			_wq_manager_should_exit.store(true);
			PX4_ERR("task start failed (%i)", task_id);
			return -errno;
		}

	} else {
		PX4_WARN("already running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
WorkQueueManagerStop()
{
	if (!_wq_manager_should_exit.load()) {

		// error can't shutdown until all WorkItems are removed/stopped
		if ((_wq_manager_wqs_list != nullptr) && (_wq_manager_wqs_list->size() > 0)) {
			PX4_ERR("can't shutdown with active WQs");
			WorkQueueManagerStatus();
			return PX4_ERROR;
		}

		// first ask all WQs to stop
		if (_wq_manager_wqs_list != nullptr) {
			{
				LockGuard lg{_wq_manager_wqs_list->mutex()};

				// ask all work queues (threads) to stop
				// NOTE: not currently safe without all WorkItems stopping first
				for (WorkQueue *wq : *_wq_manager_wqs_list) {
					wq->request_stop();
				}
			}

			// wait until they're all stopped (empty list)
			while (_wq_manager_wqs_list->size() > 0) {
				px4_usleep(1000);
			}

			delete _wq_manager_wqs_list;
		}

		_wq_manager_should_exit.store(true);

		if (_wq_manager_create_queue != nullptr) {
			// push nullptr to wake the wq manager task
			_wq_manager_create_queue->push(nullptr);

			px4_usleep(10000);

			delete _wq_manager_create_queue;
		}

	} else {
		PX4_WARN("not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int
WorkQueueManagerStatus()
{
	if (!_wq_manager_should_exit.load() && (_wq_manager_wqs_list != nullptr)) {

		const size_t num_wqs = _wq_manager_wqs_list->size();
		PX4_INFO_RAW("\nWork Queue: %-1zu threads                        RATE        INTERVAL\n", num_wqs);

		LockGuard lg{_wq_manager_wqs_list->mutex()};
		size_t i = 0;

		for (WorkQueue *wq : *_wq_manager_wqs_list) {
			i++;

			const bool last_wq = (i >= num_wqs);

			if (!last_wq) {
				PX4_INFO_RAW("|__ %zu) ", i);

			} else {
				PX4_INFO_RAW("\\__ %zu) ", i);
			}

			wq->print_status(last_wq);
		}
		/*
		这会导致  mavlink 控制台崩溃
		const wq_config_t *wq = _wq_manager_create_queue->pop();


		if (wq != nullptr) {

			PX4_INFO("_wq_manager_create_queue   != NULL");
		} else {
			PX4_INFO("_wq_manager_create_queue   NULL");
		}
		*/
		if(_wq_manager_should_exit.load()){

			PX4_INFO("test_count__   = %u",test_count__);
		} else {

		}
		PX4_INFO("test_count__   = %u",test_count__);
		PX4_INFO("test_flag_wq_manager_create_queue   = %u",test_flag_wq_manager_create_queue);
		PX4_INFO("test_WorkQueueRunner__a   = %u",test_WorkQueueRunner__a);
		PX4_INFO("test_WorkQueueRunner__b   = %u",test_WorkQueueRunner__b);
	} else {
		PX4_INFO("not running");
	}

	return PX4_OK;
}

} // namespace px4
