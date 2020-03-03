package org.poly2tri.util

class ArrayDeque<E> {

    private var elements: Array<Any?> = arrayOfNulls(16)

    private var head = 0
    private var tail = 0

    val isEmpty: Boolean
        get() = head == tail

    fun addFirst(e: E) {
        head = head - 1 and (elements.size - 1)
        elements[head] = e
        if (head == tail) doubleCapacity()
    }

    fun addLast(e: E) {
        elements[tail] = e
        tail = tail + 1 and (elements.size - 1)
        if (tail == head) doubleCapacity()
    }

    fun removeFirst(): E? {
        val h = head
        @Suppress("UNCHECKED_CAST")
        val result = elements[h] as E? ?: return null
        elements[h] = null
        head = h + 1 and elements.size - 1
        return result
    }

    private fun doubleCapacity() {
        val p = head
        val n = elements.size
        val r = n - p
        val newCapacity = n shl 1
        if (newCapacity < 0)
            throw IllegalStateException("Sorry, deque too big")
        val a = arrayOfNulls<Any>(newCapacity)
        elements.copyInto(destination = a, destinationOffset = 0, startIndex = p, endIndex = p + r)
        elements.copyInto(destination = a, destinationOffset = r, startIndex = 0, endIndex = p)
        elements = a
        head = 0
        tail = n
    }

}
