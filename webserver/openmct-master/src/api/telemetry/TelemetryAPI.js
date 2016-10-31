/*****************************************************************************
 * Open MCT, Copyright (c) 2014-2016, United States Government
 * as represented by the Administrator of the National Aeronautics and Space
 * Administration. All rights reserved.
 *
 * Open MCT is licensed under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Open MCT includes source code licensed under additional open source
 * licenses. See the Open Source Licenses file (LICENSES.md) included with
 * this source code distribution or the Licensing information page available
 * at runtime from the About dialog for additional information.
 *****************************************************************************/

define([
    'lodash',
    'EventEmitter'
], function (
    _,
    EventEmitter
) {
    /**
     * A LimitEvaluator may be used to detect when telemetry values
     * have exceeded nominal conditions.
     *
     * @interface LimitEvaluator
     * @memberof module:openmct.TelemetryAPI~
     */

    /**
     * Check for any limit violations associated with a telemetry datum.
     * @method evaluate
     * @param {*} datum the telemetry datum to evaluate
     * @param {TelemetryProperty} the property to check for limit violations
     * @memberof module:openmct.TelemetryAPI~LimitEvaluator
     * @returns {module:openmct.TelemetryAPI~LimitViolation} metadata about
     *          the limit violation, or undefined if a value is within limits
     */

    /**
     * A violation of limits defined for a telemetry property.
     * @typedef LimitViolation
     * @memberof {module:openmct.TelemetryAPI~}
     * @property {string} cssclass the class (or space-separated classes) to
     *           apply to display elements for values which violate this limit
     * @property {string} name the human-readable name for the limit violation
     */

    /**
     * A TelemetryFormatter converts telemetry values for purposes of
     * display as text.
     *
     * @interface TelemetryFormatter
     * @memberof module:openmct.TelemetryAPI~
     */

    /**
     * Retrieve the 'key' from the datum and format it accordingly to
     * telemetry metadata in domain object.
     *
     * @method format
     * @memberof module:openmct.TelemetryAPI~TelemetryFormatter#
     */




    // format map is a placeholder until we figure out format service.
    var FORMAT_MAP = {
        generic: function (range) {
            return function (datum) {
                return datum[range.key];
            };
        },
        enum: function (range) {
            var enumMap = _.indexBy(range.enumerations, 'value');
            return function (datum) {
                try {
                    return enumMap[datum[range.valueKey]].text;
                } catch (e) {
                    return datum[range.valueKey];
                }
            };
        }
    };

    FORMAT_MAP.number =
        FORMAT_MAP.float =
        FORMAT_MAP.integer =
        FORMAT_MAP.ascii =
        FORMAT_MAP.generic;

    /**
     * Describes a property which would be found in a datum of telemetry
     * associated with a particular domain object.
     *
     * @typedef TelemetryProperty
     * @memberof module:openmct.TelemetryAPI~
     * @property {string} key the name of the property in the datum which
     *           contains this telemetry value
     * @property {string} name the human-readable name for this property
     * @property {string} [units] the units associated with this property
     * @property {boolean} [temporal] true if this property is a timestamp, or
     *           may be otherwise used to order telemetry in a time-like
     *           fashion; default is false
     * @property {boolean} [numeric] true if the values for this property
     *           can be interpreted plainly as numbers; default is true
     * @property {boolean} [enumerated] true if this property may have only
     *           certain specific values; default is false
     * @property {string} [values] for enumerated states, an ordered list
     *           of possible values
     */

    /**
     * Describes and bounds requests for telemetry data.
     *
     * @typedef TelemetryRequest
     * @memberof module:openmct.TelemetryAPI~
     * @property {string} sort the key of the property to sort by. This may
     *           be prefixed with a "+" or a "-" sign to sort in ascending
     *           or descending order respectively. If no prefix is present,
     *           ascending order will be used.
     * @property {*} start the lower bound for values of the sorting property
     * @property {*} end the upper bound for values of the sorting property
     * @property {string[]} strategies symbolic identifiers for strategies
     *           (such as `minmax`) which may be recognized by providers;
     *           these will be tried in order until an appropriate provider
     *           is found
     */

    /**
     * Provides telemetry data. To connect to new data sources, new
     * TelemetryProvider implementations should be
     * [registered]{@link module:openmct.TelemetryAPI#addProvider}.
     *
     * @interface TelemetryProvider
     * @memberof module:openmct.TelemetryAPI~
     */



    /**
     * An interface for retrieving telemetry data associated with a domain
     * object.
     *
     * @interface TelemetryAPI
     * @augments module:openmct.TelemetryAPI~TelemetryProvider
     * @memberof module:openmct
     */
    function TelemetryAPI() {
        this.providersByStrategy = {};
        this.defaultProviders = [];
    }

    /**
     * Check if this provider can supply telemetry data associated with
     * this domain object.
     *
     * @method canProvideTelemetry
     * @param {module:openmct.DomainObject} domainObject the object for
     *        which telemetry would be provided
     * @returns {boolean} true if telemetry can be provided
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     */
    TelemetryAPI.prototype.canProvideTelemetry = function (domainObject) {
        return this.defaultProviders.some(function (provider) {
            return provider.canProvideTelemetry(domainObject);
        });
    };

    /**
     * Register a telemetry provider with the telemetry service. This
     * allows you to connect alternative telemetry sources.
     * @method addProvider
     * @memberof module:openmct.TelemetryAPI#
     * @param {module:openmct.TelemetryAPI~TelemetryProvider} provider the new
     *        telemetry provider
     * @param {string} [strategy] the request strategy supported by
     *        this provider. If omitted, this will be used as a
     *        default provider (when no strategy is requested or no
     *        matching strategy is found.)
     */
    TelemetryAPI.prototype.addProvider = function (provider, strategy) {
        if (!strategy) {
            this.defaultProviders.push(provider);
        } else {
            this.providersByStrategy[strategy] =
                this.providersByStrategy[strategy] || [];
            this.providersByStrategy[strategy].push(provider);
        }
    };

    /**
     * @private
     */
    TelemetryAPI.prototype.findProvider = function (domainObject, strategy) {
        function supportsDomainObject(provider) {
            return provider.canProvideTelemetry(domainObject);
        }

        if (strategy) {
            var eligibleProviders =
                (this.providersByStrategy[strategy] || [])
                    .filter(supportsDomainObject);
            if (eligibleProviders.length > 0) {
                return eligibleProviders[0];
            }
        }

        return this.defaultProviders.filter(supportsDomainObject)[0];
    };

    /**
     * Request historical telemetry for a domain object.
     * The `options` argument allows you to specify filters
     * (start, end, etc.), sort order, and strategies for retrieving
     * telemetry (aggregation, latest available, etc.).
     *
     * @method request
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     * @param {module:openmct.DomainObject} domainObject the object
     *        which has associated telemetry
     * @param {module:openmct.TelemetryAPI~TelemetryRequest} options
     *        options for this historical request
     * @returns {Promise.<object[]>} a promise for an array of
     *          telemetry data
     */
    TelemetryAPI.prototype.request = function (domainObject, options) {
        var provider = this.findProvider(domainObject, options.strategy);
        return provider ?
            provider.request(domainObject, options) :
            Promise.reject([]);
    };

    /**
     * Subscribe to realtime telemetry for a specific domain object.
     * The callback will be called whenever data is received from a
     * realtime provider.
     *
     * @method subscribe
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     * @param {module:openmct.DomainObject} domainObject the object
     *        which has associated telemetry
     * @param {Function} callback the callback to invoke with new data, as
     *        it becomes available
     * @param {module:openmct.TelemetryAPI~TelemetryRequest} options
     *        options for this request
     * @returns {Function} a function which may be called to terminate
     *          the subscription
     */

    /**
     * Get a list of all telemetry properties defined for this
     * domain object.
     *
     * @param {module:openmct.DomainObject} domainObject the domain
     *        object for which to request telemetry
     * @returns {module:openmct.TelemetryAPI~TelemetryProperty[]}
     *          telemetry metadata
     * @method properties
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     */

    /**
     * Telemetry formatters help you format telemetry values for
     * display. Under the covers, they use telemetry metadata to
     * interpret your telemetry data, and then they use the format API
     * to format that data for display.
     *
     * This method is optional.
     * If a provider does not implement this method, it is presumed
     * that all telemetry associated with this domain object can
     * be formatted correctly by string coercion.
     *
     * @param {module:openmct.DomainObject} domainObject the domain
     *        object for which to format telemetry
     * @returns {module:openmct.TelemetryAPI~TelemetryFormatter}
     * @method formatter
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     */

    /**
     * Get a limit evaluator for this domain object.
     * Limit Evaluators help you evaluate limit and alarm status of individual telemetry datums for display purposes without having to interact directly with the Limit API.
     *
     * This method is optional.
     * If a provider does not implement this method, it is presumed
     * that no limits are defined for this domain object's telemetry.
     *
     * @param {module:openmct.DomainObject} domainObject the domain
     *        object for which to evaluate limits
     * @returns {module:openmct.TelemetryAPI~LimitEvaluator}
     * @method limitEvaluator
     * @memberof module:openmct.TelemetryAPI~TelemetryProvider#
     */
    _.forEach({
        subscribe: undefined,
        properties: [],
        formatter: undefined,
        limitEvaluator: undefined
    }, function (defaultValue, method) {
        TelemetryAPI.prototype[method] = function (domainObject) {
            var provider = this.findProvider(domainObject);
            return provider ?
                provider[method].apply(provider, arguments) :
                defaultValue;
        };
    });

    return TelemetryAPI;
});
